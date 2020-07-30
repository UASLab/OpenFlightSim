#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 11:22:02 2020

@author: rega0051
"""

import os
import numpy as np

pathGoldy3 = os.path.abspath('../..')
pathRaptrs = os.path.join(pathGoldy3, 'RAPTRS')
pathRaptrsCommon = os.path.join(pathRaptrs, 'software', 'src', 'common')

# Hack to allow loading the RAPTRS package
from sys import path
path.insert(0, pathRaptrsCommon)

del path

# Hack to load FMU
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), ".")))
    path.insert(0, abspath(join(dirname(argv[0]), ".", 'python')))

    del path, argv, dirname, abspath, join


# Visualization is defined for JSBSim in the OutputFgfs.xml, Flightgear should be running prior
# Linux: ./fgfs_JSBSim.sh F450
# Windows: ./fgfs_JSBSim.bat F450


#%% JSBSim
import jsbsim as jsb
from os import path

pathJSB = '.'
fdm = jsb.FGFDMExec(pathJSB, None)

model = 'F450'

fdm.load_model(model)
fdm.set_dt(1/200)

# Load IC file
fdm.load_ic('initGrnd.xml', True)

# Setup JSBSim to FlightGear
fdm.set_output_directive(path.join('scripts', 'OutputFgfs.xml'))

# Setup JSBSim Logging
fdm.set_output_directive(path.join('scripts', 'OutputLog.xml'))


# Display the Output
i = 0
while (str(fdm.get_output_filename(i), 'utf-8') != ''):
    outStr = str(fdm.get_output_filename(i), 'utf-8')
    if '/UDP' in outStr:
        print('Output FGFS: ', outStr)
    elif '.csv' in outStr:
        fileLog = outStr
        print('Output Log: ', outStr)
    i += 1

# FDM Initialize
fdm.disable_output() # Disable Output
fdm.run_ic()

fdm['fcs/throttle-cmd-norm'] = 0.0
fdm['fcs/throttle-cmd-norm[1]'] = 0.0
fdm['fcs/throttle-cmd-norm[2]'] = 0.0
fdm['fcs/throttle-cmd-norm[3]'] = 0.0
fdm.run()
fdm.do_trim(2)
fdm.get_trim_status()

print('Alpha :', fdm['aero/alpha-deg'])

fdm.enable_output()

#%% Define Controllers
import control

tFrameRate_s = 1/50 # Desired Run rate

def PID2(Kp = 1, Ki = 0.0, Kd = 0, b = 1, c = 1, Tf = 0):
    # Inputs: ['ref', 'sens']
    # Outputs: ['cmd']

    sysR = control.tf2ss(control.tf([Kp*b*Tf + Kd*c, Kp*b + Ki*Tf, Ki], [Tf, 1, 0]))
    sysY = control.tf2ss(control.tf([Kp*Tf + Kd, Kp + Ki*Tf, Ki], [Tf, 1, 0]))

    sys = control.append(sysR, sysY)

    sys.C = sys.C[0,:] - sys.C[1,:]
    sys.D = sys.D[0,:] - sys.D[1,:]

    sys.outputs = 1

    return sys


# SCAS Controller Models
sysAlt = PID2(0.1, Ki = 0.00, Kd = 0.0, b = 1, c = 0, Tf = tFrameRate_s)
sysAlt.InputName = ['refAlt', 'sensAlt']
sysAlt.OutputName = ['refAltRate']

sysPhi = PID2(0.3, Ki = 0.0, Kd = 0.0, b = 1, c = 0, Tf = tFrameRate_s)
sysPhi.InputName = ['refPhi', 'sensPhi']
sysPhi.OutputName = ['refP']

sysTheta = PID2(0.3, Ki = 0.0, Kd = 0.0, b = 1, c = 0, Tf = tFrameRate_s)
sysTheta.InputName = ['refTheta', 'sensTheta']
sysTheta.OutputName = ['refQ']

sysPsi = PID2(0.1, Ki = 0.0, Kd = 0.0, b = 1, c = 0, Tf = tFrameRate_s)
sysPsi.InputName = ['refPsi', 'sensPsi']
sysPsi.OutputName = ['refR']


sysHeave = PID2(0.2, Ki = 0.00, Kd = 0.05, b = 1, c = 0, Tf = 1*tFrameRate_s)
sysHeave.InputName = ['refAltRate', 'sensAltRate']
sysHeave.OutputName = ['cmdHeave']

sysP = PID2(0.050, Ki = 1 * 0.015, Kd = 0.036, b = 1, c = 0, Tf = 5*tFrameRate_s)
sysP.InputName = ['refP', 'sensP']
sysP.OutputName = ['cmdP']

sysQ = PID2(0.050, Ki = 1 * 0.015, Kd = 0.036, b = 1, c = 0, Tf = 5*tFrameRate_s)
sysQ.InputName = ['refQ', 'sensQ']
sysQ.OutputName = ['cmdQ']

sysR = PID2(0.250, Ki = 1 * 0.100, Kd = 0.1, b = 1, c = 0, Tf = 5*tFrameRate_s)
sysR.InputName = ['refR', 'sensR']
sysR.OutputName = ['cmdR']

# Append systems
sysAtt = control.append(sysAlt, sysPhi, sysTheta, sysPsi)
sysAtt.InputName = sysAlt.InputName + sysPhi.InputName + sysTheta.InputName + sysPsi.InputName
sysAtt.OutputName = sysAlt.OutputName + sysPhi.OutputName + sysTheta.OutputName + sysPsi.OutputName

sysScas = control.append(sysHeave, sysP, sysQ, sysR)
sysScas.InputName = sysHeave.InputName + sysP.InputName + sysQ.InputName + sysR.InputName
sysScas.OutputName = sysHeave.OutputName + sysP.OutputName + sysQ.OutputName + sysR.OutputName

# sysScas = control.c2d(sysScas, tFrameRate_s)

# Mixer
ctrlEff = 0.25 * np.array([
    [ 1.0, 1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0,-1.0],
    [ 1.0,-1.0, 1.0,-1.0],
    [-1.0,-1.0, 1.0, 1.0]])

mixSurf = np.linalg.pinv(ctrlEff)
mixSurf [abs(mixSurf) / np.max(abs(mixSurf)) < 0.05] = 0.0


#%%
sensAlt0 = fdm['position/geod-alt-km'] * 1e3
sensPhi0 = fdm['attitude/phi-rad']
sensTheta0 = fdm['attitude/theta-rad']
sensPsi0 = fdm['attitude/psi-rad']
sensAltRate0 = fdm['velocities/h-dot-fps'] * 0.3048
sensP0 = fdm['velocities/pi-rad_sec']
sensQ0 = fdm['velocities/qi-rad_sec']
sensR0 = fdm['velocities/ri-rad_sec']

refP = sensP0
refQ = sensQ0

# Mixer Init
yMixer0 = np.array([fdm['fcs/throttle-cmd-norm'], fdm['fcs/throttle-cmd-norm[1]'], fdm['fcs/throttle-cmd-norm[2]'], fdm['fcs/throttle-cmd-norm[3]']])
uMixer0 = ctrlEff @ yMixer0

# SCAS and Att Init
xScas = np.matrix(np.zeros(sysScas.states)).T
xAtt = np.matrix(np.zeros(sysAtt.states)).T

# Simulate
# Time
tStep = np.array([[0, tFrameRate_s]])
tSamp_s = np.arange(0, 60.0, tFrameRate_s)
tFdm_s = 0.0

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
    refAlt = 0
    refPhi = 0
    refTheta = 0
    refPsi = sensPsi0
    refAltRate = 0
    refP = 0
    refQ = 0
    refR = 0

    sensAlt = fdm['position/geod-alt-km'] * 1e3
    sensPhi = fdm['attitude/phi-rad']
    sensTheta = fdm['attitude/theta-rad']
    sensPsi = fdm['attitude/psi-rad']
    sensAltRate = fdm['velocities/h-dot-fps'] * 0.3048
    sensP = fdm['velocities/pi-rad_sec']
    sensQ = fdm['velocities/qi-rad_sec']
    sensR = fdm['velocities/ri-rad_sec']

    ## Tests
    excPhi = 0
    excTheta = 0
    excP = 0
    excQ = 0
    excR = 0

    # Roll Step
    # if t_s >= 10 :
    #     excPhi = 20 *np.pi/180.0

    # P Doublet
    # if t_s >= 10 and t_s < 11 :
    #     excP = 0 + 20 *np.pi/180.0
    # elif t_s >= 11 and t_s < 12 :
    #     excP = 0 - 20 *np.pi/180.0

    # Pitch Doublet
    if t_s >= 20 and t_s < 22 :
        excTheta = 0 + 20 *np.pi/180.0
    elif t_s >= 22 and t_s < 24 :
        excTheta = 0 - 20 *np.pi/180.0

    # R Doublet
    if t_s >= 30 and t_s < 32 :
        excR = 0 + 2 *np.pi/180.0
    if t_s >= 32 and t_s < 34 :
        excR = 0 - 2 *np.pi/180.0

    ## Run Scas
    refPhi = refPhi + excPhi
    refTheta = refTheta + excTheta

    if t_s < 1.0:
        yMixer = np.array([0, 0, 0, 0])

    elif t_s >= 1.0 and t_s < 5:
        refAltRate = 2.5

        inScas = np.array([refAltRate, sensAltRate, 0, 0, 0, 0, 0, 0])
        tScas, yScas, xScas = control.forced_response(sysScas, T = tStep, U = np.array([inScas, inScas]).T, X0 = xScas[:,-1])

        # Surface Mixer
        cmdHeave, cmdP, cmdQ, cmdR = yScas[:,-1]

        uMixer = np.array([cmdHeave, cmdP, cmdQ, cmdR])
        yMixer = np.clip(mixSurf @ uMixer, 0, 1)

    elif t_s >= 5.0:
        # refAltRate = 5
        refAlt = 400

        inAtt = np.array([refAlt, sensAlt, refPhi, sensPhi, refTheta, sensTheta, refPsi, sensPsi])
        tAtt, yAtt, xAtt = control.forced_response(sysAtt, T = tStep, U = np.array([inAtt, inAtt]).T, X0 = xAtt[:,-1])

        refAltRate = yAtt[0, -1]
        refP = yAtt[1, -1] + excP
        refQ = yAtt[2, -1] + excQ

        inScas = np.array([refAltRate, sensAltRate, refP, sensP, refQ, sensQ, refR, sensR])
        tScas, yScas, xScas = control.forced_response(sysScas, T = tStep, U = np.array([inScas, inScas]).T, X0 = xScas[:,-1])

        # Surface Mixer
        cmdHeave = yScas[0, -1]
        cmdP = yScas[1, -1] + excP
        cmdQ = yScas[2, -1] + excQ
        cmdR = yScas[3, -1] + excR

        uMixer = np.array([cmdHeave, cmdP, cmdQ, cmdR])
        yMixer = np.clip(mixSurf @ uMixer, 0, 1)


    yList.append(yMixer)

    delay = 6 # frames (1 is current, 2 is 1 frame, 3 is 2 frames, ...)

    if len(yList) > delay:
        cmdMotorFR_nd, cmdMotorAL_nd, cmdMotorFL_nd, cmdMotorAR_nd = yList[-delay]
    else:
        cmdMotorFR_nd, cmdMotorAL_nd, cmdMotorFL_nd, cmdMotorAR_nd = yMixer

    ##
    # Write the Effectors
    fdm['fcs/cmdMotorFR_ext_nd'] = cmdMotorFR_nd
    fdm['fcs/cmdMotorAL_ext_nd'] = cmdMotorAL_nd
    fdm['fcs/cmdMotorFL_ext_nd'] = cmdMotorFL_nd
    fdm['fcs/cmdMotorAR_ext_nd'] = cmdMotorAR_nd

    refAltRateList.append(refAltRate)
    refPList.append(refP)
    refQList.append(refQ)
    refRList.append(refR)

    # print([t_s, sensAltRate, sensP, sensQ, sensR])
    # print([t_s, cmdMotorFR_nd, cmdMotorAL_nd, cmdMotorFL_nd, cmdMotorAR_nd])


    sensPhiList.append(sensPhi)
    sensThetaList.append(sensTheta)
    sensAltRateList.append(sensAltRate)
    sensPList.append(sensP)
    sensQList.append(sensQ)
    sensRList.append(sensR)

    # Step the FDM
    tFdm_s += tFrameRate_s
    while (fdm.get_sim_time() <= (tFdm_s - fdm.get_delta_t())): # Run the FDM
        fdm.run()

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
