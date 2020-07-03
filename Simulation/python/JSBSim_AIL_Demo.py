#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 11:22:02 2020

@author: rega0051
"""

import time
import os
import pygame
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

import control

# Visualization is defined for JSBSim in the OutputFgfs.xml, Flightgear should be running prior
# Linux: ./fgfs_JSBSim.sh UltraStick25e
# Windows: ./fgfs_JSBSim.bat UltraStick25e


#%% Joystick as SBUS source
pygame.init()

# Set up the joystick
pygame.joystick.init()

# Enumerate joysticks
joyList = []
for i in range(0, pygame.joystick.get_count()):
    joyList.append(pygame.joystick.Joystick(i).get_name())

print(joyList)

if joyList == []:
    print('Warning: joystick not found, SBUS will be zeros')

# By default, load the first available joystick.
joy = []
if (len(joyList) > 0):
    joy = pygame.joystick.Joystick(0)
    joy.init()


# Joystick Map, FIXIT - this is hacky
# OpenTX Mixer: 1-Roll, 2-Pitch, 3-Thrt, 4-Yaw, 5-SA, 6-SB, 7-SC, 8-<blank>
# SBUS def from thor.json:
def JoyMap(joy):
    msg = [0.0] * 16

    if joy != []:
        joyAxes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
        joyButtons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
        #joyHats = [joy.get_hat(i) for i in range(joy.get_numhats())]


        msg[0] = 2*joyButtons[0]-1 # Autopilot mode (-1=FMU, 1=SOC)
        msg[1] = 2*joyButtons[2]-1 # Throttle Cut
        msg[2] = 0 # RSSI
        msg[3] = joyAxes[0] # Roll
        msg[4] = joyAxes[1] # Pitch
        msg[5] = joyAxes[3] # Yaw
        msg[6] = 0 # Flap
        msg[7] = joyAxes[2] # Throttle
        msg[8] = joyAxes[4] # Control Mode
        msg[9] = joyAxes[5] # Test Select
        msg[10] = 2*joyButtons[1]-1 # Trigger (-1=Nothing, 1=Trigger)
        msg[11] = joyAxes[6] # Baseline Select

    return msg



#%% JSBSim
import jsbsim as jsb
from os import path

pathJSB = '.'
fdm = jsb.FGFDMExec(pathJSB, None)

model = 'UltraStick25e'

fdm.load_model(model)
fdm.set_dt(1/200)

# Load IC file
fdm.load_ic('initCruise.xml', True)

# Setup JSBSim to FlightGear
fdm.set_output_directive(path.join('scripts', 'OutputFgfs.xml'))

# Setup JSBSim Logging
fdm.set_output_directive(path.join('scripts', 'OutputLog.xml'))


# Display the Output
i = 0
while (str(fdm.get_output_filename(i), 'utf-8') != ''):
    outStr = str(fdm.get_output_filename(i), 'utf-8')
    if '/TCP' in outStr:
        print('Output FGFS: ', outStr)
    elif '.csv' in outStr:
        fileLog = outStr
        print('Output Log: ', outStr)
    i += 1

# FDM Initialize
fdm.disable_output() # Disable Output
fdm.run_ic()

fdm['fcs/throttle-cmd-norm'] = 0.5
fdm.run()
fdm.do_trim(1)
fdm.get_trim_status()

print('Alpha :', fdm['aero/alpha-deg'])

fdm.enable_output()

#%% Define Controllers
tFrameRate_s = 1/50 # Desired Run rate

def PID2(Kp = 1, Ki = 0.5, Kd = 0, b = 1, c = 1, Tf = 0):
    # Inputs: ['ref', 'sens']
    # Outputs: ['cmd']

    sysR = control.tf2ss(control.tf([Kp*b*Tf + Kd*c, Kp*b + Ki*Tf, Ki], [Tf, 1, 0]))
    sysY = control.tf2ss(control.tf([Kp*Tf + Kd, Kp + Ki*Tf, Ki], [Tf, 1, 0]))

    sys = control.append(sysR, sysY)

    sys.C = sys.C[0,:] - sys.C[1,:]
    sys.D = sys.D[0,:] - sys.D[1,:]

    sys.outputs = 1

    return sys

# Guidance Controller Models
sysV = PID2(0.20, Ki = 0.075, Kd = 0.0, b = 1, c = 0, Tf = tFrameRate_s)
sysV.InputName = ['refV', 'sensV']
sysV.OutputName = ['cmdV']

sysH = PID2(0.11, Ki = 0.1, Kd = 0.01, b = 1, c = 0, Tf = tFrameRate_s)
sysH.InputName = ['refH', 'sensH']
sysH.OutputName = ['refTheta']

# Append systems
sysGuid = control.append(sysV, sysH)
sysGuid.InputName = sysV.InputName + sysH.InputName
sysGuid.OutputName = sysV.OutputName + sysH.OutputName


# SCAS Controller Models
sysPhi = PID2(0.64, Ki = 0.20, Kd = 0.07, b = 1, c = 0, Tf = tFrameRate_s)
sysPhi.InputName = ['refPhi', 'sensPhi']
sysPhi.OutputName = ['cmdP']

sysTheta = PID2(0.90, Ki = 0.30, Kd = 0.08, b = 1, c = 0, Tf = tFrameRate_s)
sysTheta.InputName = ['refTheta', 'sensTheta']
sysTheta.OutputName = ['cmdQ']

tauYaw = 5.72
sysYaw = control.tf2ss(control.tf([0.03, 0.0],[1.0, tauYaw]))
sysYaw.InputName = ['sensR']
sysYaw.OutputName = ['cmdR']

# Append systems
sysScas = control.append(sysPhi, sysTheta, sysYaw)
sysScas.InputName = sysPhi.InputName + sysTheta.InputName + sysYaw.InputName
sysScas.OutputName = sysPhi.OutputName + sysTheta.OutputName + sysYaw.OutputName

# Mixer
ctrlEff = np.array([
  [0.00000,-0.14180,-1.33413,-0.56340, 0.56340, 1.33413],
  [-2.2716, 0.00000, 0.06000, 0.05800, 0.05800, 0.06000],
  [0.00000,-1.59190, 0.00000, 0.00000, 0.00000, 0.00000]])

mixSurf = np.linalg.pinv(ctrlEff)
mixSurf [abs(mixSurf) / np.max(abs(mixSurf)) < 0.05] = 0.0

sysMixerSurf = control.ss([], [], [], mixSurf)
sysMixerSurf.InputName = ['cmdP', 'cmdQ', 'cmdR']
sysMixerSurf.OutputName = ['cmdElev_rad', 'cmdRud_rad', 'cmdAilL_rad', 'cmdFlapL_rad', 'cmdFlapR_rad', 'cmdAilR_rad']


# Motor Mix
sysMixerMotor = control.ss([], [], [], 1)
sysMixerMotor.InputName = ['cmdV']
sysMixerMotor.OutputName = ['cmdMotor']

sysMixer = control.append(sysMixerMotor, sysMixerSurf)
sysMixer.InputName = sysMixerMotor.InputName + sysMixerSurf.InputName
sysMixer.OutputName = sysMixerMotor.OutputName + sysMixerSurf.OutputName

#%% Combine Controller and Mixer
connectName = sysMixer.InputName[1:]
inKeep = [sysMixer.InputName[0]] + sysScas.InputName
outKeep = sysMixer.OutputName

sysCtrl = ConnectName([sysScas, sysMixer], connectName, inKeep, outKeep)


#%%
#
kts2mps = 0.514444

refV = fdm['velocities/vtrue-kts'] * kts2mps
refH = fdm['position/geod-alt-km'] * 1000



tStart_s = time.time()
tFdm_s = 0.0

while (True):
    tFrameStart_s = time.time()

    # Read Sim Outputs (Controller inputs)
    # Read the perfect sensed values
    accelX_mps2 = fdm['sensor/imu/accelX_mps2']
    accelY_mps2 = fdm['sensor/imu/accelY_mps2']
    accelZ_mps2 = fdm['sensor/imu/accelZ_mps2']
    gyroY_rps = fdm['sensor/imu/gyroX_rps']
    gyroY_rps = fdm['sensor/imu/gyroY_rps']
    gyroY_rps = fdm['sensor/imu/gyroY_rps']


    #%%
    sensV = fdm['velocities/vtrue-kts'] * kts2mps
    excV = 0
    inGuid =
    sysGuid.InputName = sysV.InputName + sysH.InputName
    sysGuid.OutputName = sysV.OutputName + sysH.OutputName



    #%%
    # Write the Effectors
    fdm['fcs/cmdMotor_ext_nd'] = cmdMotor_nd
    fdm['fcs/cmdElev_ext_rad'] = cmdElev_rad
    fdm['fcs/cmdRud_ext_rad'] = cmdRud_rad
    fdm['fcs/cmdAilR_ext_rad'] = cmdAilR_rad
    fdm['fcs/cmdFlapR_ext_rad'] = cmdFlapR_rad
    fdm['fcs/cmdFlapL_ext_rad'] = cmdFlapL_rad
    fdm['fcs/cmdAilL_ext_rad'] = cmdAilL_rad



    # Step the FDM
    tFdm_s += tFrameRate_s
    if (fmuMode is 'Run'):
        while (fdm.get_sim_time() <= (tFdm_s - fdm.get_delta_t())): # Run the FDM
            fdm.run()

    # Timer, do not expect realtime
    tHold_s = tFrameRate_s - (time.time() - tFrameStart_s)
    if tHold_s > 0:
        time.sleep(tHold_s)

# end while(True)

#SocComms.Close()


