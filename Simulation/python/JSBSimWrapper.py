#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Copyright (c) 2016 - 2020 Regents of the University of Minnesota.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
'''

#%% JSBSim
import numpy as np
import jsbsim as jsb

ft2m = 0.3048
m2ft = 1/ft2m
rad2deg = 180 / np.pi
deg2rad = 1/rad2deg
        
class JSBSimWrap():
    def __init__ (self, model, pathJSB = '.', dt = 1/200):
        
        self.model = model
        self.dt = dt
        self.pathJSB = pathJSB
        
        self.fdm = jsb.FGFDMExec(pathJSB, None)
        
        self.fdm.load_model(self.model)
        self.fdm.set_dt(self.dt)
        
        self.fileLog = []

    def SetupIC(self, icFile):
        # Load IC file
        self.fdm.load_ic(icFile, True)
        
    def SetupOutput(self, outList = ['scripts/OutputFgfs.xml', 'scripts/OutputLog.xml']):
        for outFile in outList:
            self.fdm.set_output_directive(outFile)
    
    def DispOutput(self):
        # Display the Output
        i = 0
        while (str(self.fdm.get_output_filename(i), 'utf-8') != ''):
            outStr = str(self.fdm.get_output_filename(i), 'utf-8')
            if '/UDP' in outStr:
                print('Output FGFS: ', outStr)
            elif '.csv' in outStr:
                self.fileLog = outStr
                print('Output Log: ', outStr)
            i += 1

    def RunTrim(self, trimType = 2, throttle = 0.0):
        # FDM Initialize
        self.fdm.disable_output() # Disable Output
        self.fdm.run_ic()

        for i in range(len(self.fdm.query_property_catalog('fcs/throttle-cmd-norm'))):
            self.fdm['fcs/throttle-cmd-norm[' + str(i) + ']'] = throttle

        self.fdm.run()
        self.fdm.do_trim(trimType)
        self.fdm.get_trim_status()
        
        self.fdm.enable_output()
        
    def DispTrim(self):
        print('Ax :', self.fdm['sensor/imu/accelX_mps2'])
        print('Ay :', self.fdm['sensor/imu/accelY_mps2'])
        print('Az :', self.fdm['sensor/imu/accelZ_mps2'])
        
        print('Phi :', self.fdm['attitude/phi-deg'])
        print('Theta :', self.fdm['attitude/theta-deg'])
        print('Psi :', self.fdm['attitude/psi-deg'])
        print('Alpha :', self.fdm['aero/alpha-deg'])
        
        
    def SetWindNED(self, vWind_mps = [0.0, 0.0, 0.0]):
        
        self.vWind_mps = vWind_mps
        
        self.fdm['atmosphere/wind-north-fps'] = vWind_mps[0] * m2ft
        self.fdm['atmosphere/wind-east-fps'] = vWind_mps[1] * m2ft
        self.fdm['atmosphere/wind-down-fps'] = vWind_mps[2] * m2ft
    
    def GetWindNED(self):
        
        self.vWind_mps = [self.fdm['atmosphere/wind-north-fps'] * ft2m, self.fdm['atmosphere/wind-east-fps'] * ft2m, self.fdm['atmosphere/wind-down-fps'] * ft2m]
        
        return self.vWind_mps
        
    def SetWind(self, vWindMag_mps = 0.0, vWindHeading_deg = 0.0, vWindDown_mps = 0.0):
        
        self.fdm['atmosphere/wind-mag-fps'] = vWindMag_mps * m2ft
        self.fdm['atmosphere/psiw-rad'] = vWindHeading_deg * deg2rad
        self.fdm['atmosphere/wind-down-fps'] = vWindDown_mps * m2ft
        
    # Update the wind in JSBSim for the current altitude
    def UpdateWind(self):
        vWind20_fps = self.fdm['atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps']
        h_ft = np.max([self.fdm['position/h-agl-ft'], 0.1])
        
        # Compute Wind Shear (MIL-DTL-9490E, 3.1.3.7.3.2) to compute
        self.fdm['atmosphere/wind-mag-fps'] = vWind20_fps * (0.46 * np.log10(h_ft) + 0.4)

        self.vWind20_mps = vWind20_fps * ft2m
        self.vWind_mps = self.GetWindNED()
        
    def SetTurb(self, turbType = 4, turbSeverity = 0, vWind20_mps = None, vWindHeading_deg = 0.0):
        self.vWind20_mps = vWind20_mps
        
        self.fdm['atmosphere/turb-type'] = turbType
        self.fdm['atmosphere/turbulence/milspec/severity'] = turbSeverity
        self.fdm['atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps'] = vWind20_mps * m2ft
        self.fdm['atmosphere/psiw-rad'] = vWindHeading_deg * deg2rad
        self.UpdateWind()

    def RunTo(self, time_s, updateWind = None):
            
        while (self.fdm.get_sim_time() <= time_s): # Run the FDM
            if updateWind ==  True:
                self.UpdateWind()
            self.fdm.run()
            
    def __del__(self):
        del self.fdm
        
