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


# Attitude Controller Models
sysAlt = PID2(0.100, Ki = 0.005, Kd = 0.000, b = 1, c = 0, Tf = tFrameRate_s)
sysAlt.InputName = ['refAlt', 'sensAlt']
sysAlt.OutputName = ['refAltRate']    

sysPhi = PID2(0.200, Ki = 0.010, Kd = 0.000, b = 1, c = 0, Tf = tFrameRate_s)
sysPhi.InputName = ['refPhi', 'sensPhi']
sysPhi.OutputName = ['refP']

sysTheta = PID2(0.200, Ki = 0.010, Kd = 0.000, b = 1, c = 0, Tf = tFrameRate_s)
sysTheta.InputName = ['refTheta', 'sensTheta']
sysTheta.OutputName = ['refQ']

sysPsi = PID2(0.200, Ki = 0.000, Kd = 0.000, b = 1, c = 0, Tf = tFrameRate_s)
sysPsi.InputName = ['refPsi', 'sensPsi']
sysPsi.OutputName = ['refR']

# Append Attitude systems
sysAtt = control.append(sysAlt, sysPhi, sysTheta, sysPsi)
sysAtt.InputName = sysAlt.InputName + sysPhi.InputName + sysTheta.InputName + sysPsi.InputName
sysAtt.OutputName = sysAlt.OutputName + sysPhi.OutputName + sysTheta.OutputName + sysPsi.OutputName


# SCAS Controller Models
sysAltRate = PID2(0.075, Ki = 0.000, Kd = 0.020, b = 1, c = 0, Tf = tFrameRate_s)
sysAltRate.InputName = ['refAltRate', 'sensAltRate']
sysAltRate.OutputName = ['cmdHeave']

sysP = PID2(0.150, Ki = 0.020, Kd = 0.040, b = 1, c = 0, Tf = tFrameRate_s)
sysP.InputName = ['refP', 'sensP']
sysP.OutputName = ['cmdP']

sysQ = PID2(0.150, Ki = 0.020, Kd = 0.040, b = 1, c = 0, Tf = tFrameRate_s)
sysQ.InputName = ['refQ', 'sensQ']
sysQ.OutputName = ['cmdQ']


sysR = PID2(5* 0.150, Ki = 5* 0.020, Kd = 5* 0.040, b = 1, c = 0, Tf = tFrameRate_s)
sysR.InputName = ['refR', 'sensR']
sysR.OutputName = ['cmdR']

# Append SCAS systems
sysScas = control.append(sysAltRate, sysP, sysQ, sysR)
sysScas.InputName = sysAltRate.InputName + sysP.InputName + sysQ.InputName + sysR.InputName
sysScas.OutputName = sysAltRate.OutputName + sysP.OutputName + sysQ.OutputName + sysR.OutputName

# sysScas = control.c2d(sysScas, tFrameRate_s)

# Mixer
ctrlEff = 0.25 * np.array([
    [ 1.0, 1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0,-1.0],
    [ 1.0,-1.0, 1.0,-1.0],
    [ 1.0, 1.0,-1.0,-1.0]])

mixSurf = np.linalg.pinv(ctrlEff)
mixSurf [abs(mixSurf) / np.max(abs(mixSurf)) < 0.05] = 0.0


#%%
## Load Sim
model = 'F450'
sim = JSBSimWrap(model, dt = 1/200)
sim.SetupIC('initGrnd.xml')
sim.SetupOutput()
sim.DispOutput()
sim.RunTrim()

sim.SetTurb(turbType = 4, turbSeverity = 3, vWind20_mps = 6.0, vWindHeading_deg = 0.0)

#%%
ft2m = 0.3048
##
sensAlt0 = sim.fdm['position/geod-alt-km'] * 1e3
sensPhi0 = sim.fdm['attitude/phi-rad']
sensTheta0 = sim.fdm['attitude/theta-rad']
sensPsi0 = sim.fdm['attitude/psi-rad']
sensAltRate0 = sim.fdm['velocities/h-dot-fps'] * ft2m
sensP0 = sim.fdm['sensor/imu/gyroX_rps']
sensQ0 = sim.fdm['sensor/imu/gyroY_rps']
sensR0 = sim.fdm['sensor/imu/gyroZ_rps']

refP = sensP0
refQ = sensQ0

# Mixer Init
yMixer0 = np.array([sim.fdm['fcs/throttle-cmd-norm'], sim.fdm['fcs/throttle-cmd-norm[1]'], sim.fdm['fcs/throttle-cmd-norm[2]'], sim.fdm['fcs/throttle-cmd-norm[3]']])
uMixer0 = ctrlEff @ yMixer0

# SCAS and Att Init
xScas = np.matrix(np.zeros(sysScas.states)).T
xAtt = np.matrix(np.zeros(sysAtt.states)).T

# Simulate
# Time
tStep = np.array([[0, tFrameRate_s]])
tSamp_s = np.arange(0, 40.0, tFrameRate_s)

refAltRateList = []
refPList = []
refQList = []
refRList = []

sensPhiList = []
sensThetaList = []
sensAltRateList = []
sensPList = []
sensQList = []
sensRList = []
yList = []

for t_s in tSamp_s:
    # Read Sensors
    refAlt = sensAlt0
    refPhi = 0
    refTheta = 0
    refPsi = sensPsi0
    refAltRate = 0
    refP = 0
    refQ = 0
    refR = 0

    sensAlt = sim.fdm['position/geod-alt-km'] * 1e3
    sensPhi = sim.fdm['attitude/phi-rad']
    sensTheta = sim.fdm['attitude/theta-rad']
    sensPsi = sim.fdm['attitude/psi-rad']
    sensAltRate = sim.fdm['velocities/h-dot-fps'] * ft2m
    sensP = sim.fdm['sensor/imu/gyroX_rps']
    sensQ = sim.fdm['sensor/imu/gyroY_rps']
    sensR = sim.fdm['sensor/imu/gyroZ_rps']

    ## Tests
    excRefP = 0
    excRefQ = 0
    excRefR = 0
    excCmdP = 0
    excCmdQ = 0
    excCmdR = 0

    # Psi Step
    if t_s >= 10 :
        refPsi = 45.0 *np.pi/180.0

    # Phi Step
    if t_s >= 20 and t_s < 30 :
        refPhi = 20 *np.pi/180.0

    ## Run Scas
    if t_s < 1.0:
        yMixer = np.array([0, 0, 0, 0])

    elif t_s >= 1.0 and t_s < 5:
        refAltRate = 3.0

        inScas = np.array([refAltRate, sensAltRate, 0, 0, 0, 0, 0, 0])
        tScas, yScas, xScas = control.forced_response(sysScas, T = tStep, U = np.array([inScas, inScas]).T, X0 = xScas[:,-1])

        # Surface Mixer
        cmdHeave, cmdP, cmdQ, cmdR = yScas[:,-1]

        uMixer = np.array([cmdHeave, cmdP, cmdQ, cmdR])
        yMixer = np.clip(mixSurf @ uMixer, 0, 1)
        
#        sensAlt0 = sensAlt

    elif t_s >= 5:
        refAlt = sensAlt0 + 50 * ft2m
        refPhi = refPhi
        refTheta = refTheta
        refPsi = refPsi

        inAtt = np.array([refAlt, sensAlt, refPhi, sensPhi, refTheta, sensTheta, refPsi, sensPsi])
        tAtt, yAtt, xAtt = control.forced_response(sysAtt, T = tStep, U = np.array([inAtt, inAtt]).T, X0 = xAtt[:,-1])

        if 1: # Switch to use Att or just SCAS
            refAltRate = 2.0
#            refAltRate = yAtt[0, -1]
            refP = yAtt[1, -1] + excRefP
            refQ = yAtt[2, -1] + excRefQ
            refR = yAtt[3, -1] + excRefR
        else:
            refAltRate = 2.0
            refP = excRefP
            refQ = excRefQ
            refR = excRefR

        inScas = np.array([refAltRate, sensAltRate, refP, sensP, refQ, sensQ, refR, sensR])
        tScas, yScas, xScas = control.forced_response(sysScas, T = tStep, U = np.array([inScas, inScas]).T, X0 = xScas[:,-1])

        # Surface Mixer
        cmdHeave = yScas[0, -1]
        cmdP = yScas[1, -1] + excCmdP
        cmdQ = yScas[2, -1] + excCmdQ
        cmdR = yScas[3, -1] + excCmdR

        uMixer = np.array([cmdHeave, cmdP, cmdQ, cmdR])
        yMixer = np.clip(mixSurf @ uMixer, 0, 1)

    yList.append(yMixer)
    cmdMotorFR_nd, cmdMotorAL_nd, cmdMotorFL_nd, cmdMotorAR_nd = yMixer

    ##
    # Write the Effectors
    sim.fdm['fcs/cmdMotorFR_ext_nd'] = cmdMotorFR_nd
    sim.fdm['fcs/cmdMotorAL_ext_nd'] = cmdMotorAL_nd
    sim.fdm['fcs/cmdMotorFL_ext_nd'] = cmdMotorFL_nd
    sim.fdm['fcs/cmdMotorAR_ext_nd'] = cmdMotorAR_nd

    refAltRateList.append(refAltRate)
    refPList.append(refP)
    refQList.append(refQ)
    refRList.append(refR)


    sensPhiList.append(sensPhi)
    sensThetaList.append(sensTheta)
    sensAltRateList.append(sensAltRate)
    sensPList.append(sensP)
    sensQList.append(sensQ)
    sensRList.append(sensR)

    # Step the FDM
    tFdm_s = sim.RunTo(t_s + tFrameRate_s - sim.fdm.get_delta_t(), updateWind = True)


#%%
import matplotlib.pyplot as plt
plt.figure(1)
plt.subplot(4,1,1)
plt.plot(tSamp_s, np.array(refAltRateList))
plt.plot(tSamp_s, np.array(sensAltRateList))
plt.subplot(4,1,2)
plt.plot(tSamp_s, np.array(refPList) * 180/np.pi)
# plt.plot(tSamp_s, np.array(sensPhiList) * 180/np.pi)
plt.plot(tSamp_s, np.array(sensPList) * 180/np.pi)
plt.subplot(4,1,3)
plt.plot(tSamp_s, np.array(refQList) * 180/np.pi)
# plt.plot(tSamp_s, np.array(sensThetaList) * 180/np.pi)
plt.plot(tSamp_s, np.array(sensQList) * 180/np.pi)
plt.subplot(4,1,4)
plt.plot(tSamp_s, np.array(refRList) * 180/np.pi)
plt.plot(tSamp_s, np.array(sensRList) * 180/np.pi)
plt.show()

#%%
yArray = np.array(yList)

plt.figure(2)
plt.subplot(2,2,1)
plt.plot(tSamp_s, yArray[:,2])
plt.subplot(2,2,2)
plt.plot(tSamp_s, yArray[:,0])
plt.subplot(2,2,3)
plt.plot(tSamp_s, yArray[:,1])
plt.subplot(2,2,4)
plt.plot(tSamp_s, yArray[:,3])
plt.show()
