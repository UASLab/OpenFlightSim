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


import fmu_messages
from FMU import AircraftSocComms

# Act as the FMU side of the FMU-SOC comms.

# Start a Virtual link for a psuedo-tty through TCP
#On BBB or Linux: ./start_CommSOC.sh

# Try to detect how the system is setup, SIL or PIL
if os.path.exists('ptySimSoc') == True:
    if os.path.exists('ptySimFmu') == True: # If both PTY exist then use it as a direct bridge
        host = None
        port = 'ptySimFmu'
        print('Running in SIL-PTY Mode')
    else: # The SOC PTY is present, since the FMU PTY is not present assume its on TCP localhost
        host = 'localhost'
        port = 59600
        print('Running in SIL Mode: host = localhost')
else:
    host = '192.168.7.2'
    port = 59600
    print('Running in PIL/HIL Mode: host = 192.168.7.2')


# Visualization is defined for JSBSim in the OutputFgfs.xml, Flightgear should be running prior
# Linux: ./fgfs_JSBSim.sh F450
# Windows: ./fgfs_JSBSim.bat F450
    
import psutil
if 'fgfs' in [p.name() for p in psutil.process_iter()]:
    visuals = True
else:
    visuals = False

# Once this script is running, and waiting...
# Linux: flight_amd64 thor.json


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

model = 'F450'

fdm.load_model(model)
fdm.set_dt(1/200)

# Load IC file
fdm.load_ic('initGrnd.xml', True)

# Setup JSBSim to FlightGear
if visuals == True:
    fdm.set_output_directive(path.join('scripts', 'OutputFgfs.xml'))

# Setup JSBSim Logging
fdm.set_output_directive(path.join('scripts', 'OutputLog.xml'))


# Display the Output
i = 0
while (str(fdm.get_output_filename(i),'utf-8') != ''):
    outStr = str(fdm.get_output_filename(i),'utf-8')
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
fdm.run()
fdm.do_trim(2)
fdm.get_trim_status()

print('Theta :', fdm['attitude/theta-deg'])

fdm.enable_output()

#%%
SocComms = AircraftSocComms(host, port)
SocComms.Begin()

fmuMode = 'Config'

# Data Messages
dataMsgCommand = fmu_messages.command_effectors()
dataMsgBifrost = fmu_messages.data_bifrost()

tStart_s = time.time()
tFdm_s = 0.0
tFrameRate_s = 1/50 # Desired Run rate

while (True):
    tFrameStart_s = time.time()
    time_s = tFrameStart_s - tStart_s
    
    # Update Mission run mode
    # FIXIT - No FMU Controller or Mission emulated in Python
    
    #
    if (fmuMode is 'Run'):
        # Read all the joystick values
        pygame.event.get(pump = True) # Pump, retreive events so they clear
        joyVals = JoyMap(joy)
        
        SocComms.SendSensorMessages(time_s, fdm, joyVals)
    
    # Check and Recieve Messages
    while(SocComms.CheckMessage()):
        msgID, msgAddress, msgPayload = SocComms.ReceiveMessage()
        
        # Message did not have an ID, likely a SerialLink Ack/Nack message
        if msgID is None:
            continue
        
        if msgID == fmu_messages.command_mode_id: # request mode
                    
            fmuModeMsg = fmu_messages.command_mode()
            fmuModeMsg.unpack(msg = msgPayload[-1].to_bytes(1, byteorder = 'little'))
            
            if fmuModeMsg.mode == 0:
                fmuMode = 'Config'
            elif fmuModeMsg.mode == 1:
                fmuMode = 'Run'
                    
            print ('Set FMU Mode: ' + fmuMode)
                
        elif (fmuMode is 'Run'):
            # Receive Command Effectors
            if msgID == dataMsgCommand.id:
                dataMsgCommand.unpack(msg = msgPayload)

                # FIXIT - This may be pretty fragile for general use
                # Create the list of effector fields from the FDM.
                import difflib
                if SocComms.effListFdm == []:
                    SocComms.effListFdm = []
                    for eff in SocComms.effList:
                        effName = eff.split('/')[-1]
                        match = difflib.get_close_matches(effName, fdm.query_property_catalog('_ext_'))[0]
                        
                        SocComms.effListFdm.append(match.strip(' (RW)'))
                    print(SocComms.effListFdm)
                
                # Populate the FDM commands from the dataMsgCommand.command values
                for iEff, eff in enumerate(SocComms.effListFdm):
                    fdm[eff] = dataMsgCommand.command[iEff]

#                print(dataMsgCommand.command)
            elif msgID == dataMsgBifrost.id: # FIXIT -
                pass
                # dataMsgBifrost.unpack(msg = msgPayload)
                 
        elif (fmuMode is 'Config'):
            # Parse Message as a config message
            SocComms.Config(msgID, msgPayload) 
            
    
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


