#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 11:22:02 2020

@author: rega0051
"""

import numpy as np

# Hack to load OpenFlightSim Modules
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), ".")))
    path.insert(0, abspath(join(dirname(argv[0]), ".", 'python')))

    del path, argv, dirname, abspath, join

from JSBSimWrapper import JSBSimWrap

# Visualization is defined for JSBSim in the OutputFgfs.xml, Flightgear should be running prior
# Linux: ./fgfs_JSBSim.sh {model}
# Windows: ./fgfs_JSBSim.bat {model}


#%% Define Controllers
import control

tFrameRate_s = 1/50 # Desired Run rate

def PID2(Kp = 1, Ki = 0.0, Kd = 0, b = 1, c = 1, Tf = 0, dt = None):
    # Inputs: ['ref', 'sens']
    # Outputs: ['cmd']

    sysR = control.tf2ss(control.tf([Kp*b*Tf + Kd*c, Kp*b + Ki*Tf, Ki], [Tf, 1, 0]))
    sysY = control.tf2ss(control.tf([Kp*Tf + Kd, Kp + Ki*Tf, Ki], [Tf, 1, 0]))

    sys = control.append(sysR, sysY)

    sys.C = sys.C[0,:] - sys.C[1,:]
    sys.D = sys.D[0,:] - sys.D[1,:]

    sys.outputs = 1
    
    if dt is not None:
        sys = control.c2d(sys, dt)

    return sys


# SCAS Controller Models
sysPhi = PID2(0.64, Ki = 0.20, Kd = 0.07, b = 1, c = 0, Tf = tFrameRate_s)
sysPhi.InputName = ['refPhi', 'sensPhi']
sysPhi.OutputName = ['cmdP']

sysTheta = PID2(0.90, Ki = 0.30, Kd = 0.08, b = 1, c = 0, Tf = tFrameRate_s)
sysTheta.InputName = ['refTheta', 'sensTheta']
sysTheta.OutputName = ['cmdQ']

tauYaw = 5.72
sysYaw = control.tf2ss(control.tf([0.03, 0.0], [1.0, tauYaw]))
sysYaw.InputName = ['sensR']
sysYaw.OutputName = ['cmdR']

# Append systems
sysScas = control.append(sysPhi, sysTheta, sysYaw)
sysScas.InputName = sysPhi.InputName + sysTheta.InputName + sysYaw.InputName
sysScas.OutputName = sysPhi.OutputName + sysTheta.OutputName + sysYaw.OutputName

#sysScas = control.c2d(sysScas, tFrameRate_s)

# Mixer
ctrlEff = np.array([
  [0.00000,-0.14180,-1.33413,-0.56340, 0.56340, 1.33413],
  [-2.2716, 0.00000, 0.06000, 0.05800, 0.05800, 0.06000],
  [0.00000,-1.59190, 0.00000, 0.00000, 0.00000, 0.00000]])

mixSurf = np.linalg.pinv(ctrlEff)
mixSurf [abs(mixSurf) / np.max(abs(mixSurf)) < 0.05] = 0.0


#%%
## Load Sim
model = 'UltraStick25e'
sim = JSBSimWrap(model)
sim.SetupIC('initCruise.xml')
sim.SetupOutput()
sim.DispOutput()
sim.RunTrim(trimType = 0, throttle = 0.5)
sim.DispTrim()

sim.SetTurb(turbType = 4, turbSeverity = 1, vWind20_mps = 5.0, vWindHeading_deg = 0.0)

##
sensPhi0 = sim.fdm['attitude/phi-rad']
sensTheta0 = sim.fdm['attitude/theta-rad']
sensR0 = sim.fdm['velocities/r-rad_sec']

refPhi = sensPhi0
refTheta = sensTheta0

# Mixer Init
yMixer0 = np.array([sim.fdm['fcs/cmdElev_rad'], sim.fdm['fcs/cmdRud_rad'], sim.fdm['fcs/cmdAilR_rad'], sim.fdm['fcs/cmdFlapR_rad'], sim.fdm['fcs/cmdFlapL_rad'], sim.fdm['fcs/cmdAilL_rad']])
uMixer0 = ctrlEff @ yMixer0

cmdMotor0 = np.array([sim.fdm['fcs/throttle-pos-norm']])

# SCAS Init
xScas = np.matrix(np.zeros(sysScas.states)).T

# Simulate
# Time
tStep = np.array([[0, tFrameRate_s]])
tSamp_s = np.arange(0, 20, tFrameRate_s)

refPhiList = []
refThetaList = []

sensPhiList = []
sensThetaList = []
sensVList = []

for t_s in tSamp_s:
    # Read Sensors
    refPhi = 0
    refTheta = sensTheta0

    sensPhi = sim.fdm['attitude/phi-rad']
    sensTheta = sim.fdm['attitude/theta-rad']
    sensR = sim.fdm['velocities/r-rad_sec']

    # Roll Doublet
#    if t_s < 2:
#        refPhi = 0
#    elif t_s < 4:
#        refPhi = 0 + 20 *np.pi/180.0
#    elif t_s < 6:
#        refPhi = 0 - 20 *np.pi/180.0
#    else:
#        refPhi = 0

    # Pitch Doublet
    if t_s < 10:
        refTheta = sensTheta0
    elif t_s < 12:
        refTheta = sensTheta0 + 5 *np.pi/180.0
    elif t_s < 14:
        refTheta = sensTheta0 - 5 *np.pi/180.0
    else:
        refTheta = sensTheta0

    ## Run Scas
    inScas = np.array([refPhi, sensPhi, refTheta, sensTheta, sensR])
    tScas, yScas, xScas = control.forced_response(sysScas, T = tStep, U = np.array([inScas, inScas]).T, X0 = xScas[:,-1])

    # Surface Mixer
    cmdP, cmdQ, cmdR = yScas[:,-1]

    uMixer = np.array([cmdP, cmdQ, cmdR])
    yMixer = mixSurf @ uMixer
    cmdElev_rad, cmdRud_rad, cmdAilR_rad, cmdFlapR_rad, cmdFlapL_rad, cmdAilL_rad = yMixer

    # Thurst Mixer
    cmdMotor_nd = cmdMotor0

    ##
    # Write the Effectors
    sim.fdm['fcs/throttle-cmd-norm'] = cmdMotor_nd
    sim.fdm['fcs/cmdElev_ext_rad'] = cmdElev_rad
    sim.fdm['fcs/cmdRud_ext_rad'] = cmdRud_rad
    sim.fdm['fcs/cmdAilR_ext_rad'] = cmdAilR_rad
    sim.fdm['fcs/cmdFlapR_ext_rad'] = cmdFlapR_rad
    sim.fdm['fcs/cmdFlapL_ext_rad'] = cmdFlapL_rad
    sim.fdm['fcs/cmdAilL_ext_rad'] = cmdAilL_rad


    refPhiList.append(refPhi)
    sensPhiList.append(sensPhi)

    refThetaList.append(refTheta)
    sensThetaList.append(sensTheta)

    sensVList.append(sim.fdm['velocities/vt-fps'] * 0.3048)

    # Step the FDM
    tFdm_s = sim.RunTo(t_s + tFrameRate_s - sim.fdm.get_delta_t(), updateWind = True)


#%%
import matplotlib.pyplot as plt
#plt.figure(1)
plt.subplot(3,1,1)
plt.plot(tSamp_s, np.array(refPhiList) * 180/np.pi)
plt.plot(tSamp_s, np.array(sensPhiList) * 180/np.pi)
plt.subplot(3,1,2)
plt.plot(tSamp_s, np.array(refThetaList) * 180/np.pi)
plt.plot(tSamp_s, np.array(sensThetaList) * 180/np.pi)
plt.subplot(3,1,3)
plt.plot(tSamp_s, np.array(sensVList))
