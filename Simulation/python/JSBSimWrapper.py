#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Copyright (c) 2016 - 2020 Regents of the University of Minnesota.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
'''

#%% JSBSim
import jsbsim as jsb

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

    def RunTrim(self, throttle = 0.0):
        # FDM Initialize
        self.fdm.disable_output() # Disable Output
        self.fdm.run_ic()

        for i in range(len(self.fdm.query_property_catalog('fcs/throttle-cmd-norm'))):
            self.fdm['fcs/throttle-cmd-norm[' + str(i) + ']'] = throttle

        self.fdm.run()
        self.fdm.do_trim(2)
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
        

    def RunTo(self, time_s):
        while (self.fdm.get_sim_time() <= time_s): # Run the FDM
            self.fdm.run()
            
    def __del__(self):
        del self.fdm
        
