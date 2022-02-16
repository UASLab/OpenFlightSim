#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 23 11:22:02 2020

@author: rega0051
"""

import time
import os

pathGoldy3 = os.path.abspath('../..')
pathRaptrs = os.path.join(pathGoldy3, 'RAPTRS')
pathRaptrsCommon = os.path.join(pathRaptrs, 'software', 'src', 'common')

# Hack to allow loading the RAPTRS package
from sys import path
path.insert(0, pathRaptrsCommon)
del path

# Hack to load OpenFlightSim Modules
if __name__ == "__main__" and __package__ is None:
    from sys import path, argv
    from os.path import dirname, abspath, join

    path.insert(0, abspath(join(dirname(argv[0]), ".")))
    path.insert(0, abspath(join(dirname(argv[0]), ".", 'python')))

    del path, argv, dirname, abspath, join


import fmu_messages
from FMU import AircraftSocComms
from FMU import Joystick
from JSBSimWrapper import JSBSimWrap

# Act as the FMU side of the FMU-SOC comms.

# Start a Virtual link for a psuedo-tty through TCP
#On BBB or Linux: ./start_CommSOC.sh

runmode = 'SIL-TCP'
if runmode == 'SIL-PTY': # SIL Direct PTY-PTY
    host = None
    port = 'ptySimFmu'
    print('Running in SIL-PTY Mode')
elif runmode == 'SIL-TCP': # SIL via TCP on localhost
    host = 'localhost'
    port = 59600
    print('Running in SIL Mode: host = localhost')
elif runmode == 'PIL': # PIL/HIL via TCP
    # PIL
    host = '192.168.7.2'
    port = 59600
    print('Running in PIL/HIL Mode: host = 192.168.7.2')


# Visualization is defined for JSBSim in the OutputFgfs.xml, Flightgear should be running prior
# Linux: ./fgfs_JSBSim.sh {model}
# Windows: ./fgfs_JSBSim.bat {model}

import psutil
if 'fgfs' in [p.name() for p in psutil.process_iter()]:
    visuals = True
elif 'fgfs.exe' in [p.name() for p in psutil.process_iter()]:
    visuals = True
else:
    visuals = False

# Once this script is running, and waiting...
# Linux: flight_amd64 thor.json


#%% Joystick as SBUS source
# Joystick Map, FIXIT - this is hacky
# OpenTX Mixer: 1-Roll, 2-Pitch, 3-Thrt, 4-Yaw, 5-SA, 6-SB, 7-SC, 8-<blank>
# SBUS def from thor.json:

joystick = Joystick()

# Run the joustick just to populate messages
joystick.update()
joystick.sbus()


#%% JSBSim
## Load Sim
model = 'F450'
sim = JSBSimWrap(model)
sim.SetupIC('initGrnd.xml')
sim.SetupOutput()
sim.DispOutput()
sim.RunTrim()

sim.SetTurb(turbType = 4, turbSeverity = 6, vWind20_mps = 6.0, vWindHeading_deg = 0.0)

# Start Comms with SOC
SocComms = AircraftSocComms(host, port)
SocComms.Begin()

# Initial Mode
fmuMode = 'Config'

# Data Messages
dataMsgCommand = fmu_messages.command_effectors()
dataMsgBifrost = fmu_messages.data_bifrost()

# Run Simulation
tStart_s = time.time()
tSend_s = 0.0
fdmStart = False

tFrameRate_s = 1/50 # Desired Run rate
while (True):
    time_s = time.time() - tStart_s
    # Update Mission run mode
    # FIXIT - No FMU Controller or Mission emulated in Python

    # Send Sensor data on tFrameRate_s intervals, apply a little tweak to account for time to send packets
    if (fmuMode == 'Run') and (time_s - tSend_s >= tFrameRate_s - 0.2e-3):
        tSend_s = time.time() - tStart_s

        # Read all the joystick values
        joystick.update()
        joyMsg = joystick.sbus()

        # Send all the sensor messages
        SocComms.SendSensorMessages(time_s, sim.fdm, joyMsg)

    # Check and Recieve Messages
    while(SocComms.CheckMessage()):
        msgID, msgAddress, msgPayload = SocComms.ReceiveMessage()

        # Message did not have an ID, likely a SerialLink Ack/Nack message
        if msgID is None:
            continue

        if msgID == fmu_messages.command_mode_id: # request mode change

            fmuModeMsg = fmu_messages.command_mode()
            fmuModeMsg.unpack(msg = msgPayload[-1].to_bytes(1, byteorder = 'little'))

            if fmuModeMsg.mode == 0:
                fmuMode = 'Config'
            elif fmuModeMsg.mode == 1:
                fmuMode = 'Run'

            print ('Set FMU Mode: ' + fmuMode)

        elif (fmuMode == 'Run'):
            # Receive Effector Commands
            if msgID == dataMsgCommand.id:
                tReceive_s = time.time() - tStart_s
                dataMsgCommand.unpack(msg = msgPayload)

                # FIXIT - This may be pretty fragile for general use
                # Create the list of effector fields from the FDM.
                # This needs to run after all the effector config messages have been received.
                import difflib
                if SocComms.effListFdm == []:
                    for eff in SocComms.effList:
                        effName = eff.split('/')[-1]
                        matchList = difflib.get_close_matches(effName.replace('_', '_ext_'), sim.fdm.query_property_catalog('_ext_'))
                        if matchList != []:
                            effNameFdm = matchList[0].strip(' (RW)') # First is best
                            SocComms.effListFdm.append(effNameFdm)
                    print(SocComms.effListFdm)

                # Populate the FDM commands from the dataMsgCommand.command values
                for iEff, eff in enumerate(SocComms.effListFdm):
                    sim.fdm[eff] = dataMsgCommand.command[iEff]

                # print(time_s, tSend_s, tReceive_s, dataMsgCommand.command[0:dataMsgCommand.num_active])
            elif msgID == dataMsgBifrost.id: # FIXIT -
                pass
                # dataMsgBifrost.unpack(msg = msgPayload)

        elif (fmuMode == 'Config'):
            # Parse Message as a config message
            SocComms.Config(msgID, msgPayload)

    # Start the FDM
    if fdmStart == False:
        sim.fdm.set_sim_time(time_s)
        fdmStart = True

    # Step the FDM
    # FDM should run at least to the current time, catch-up if required
    tFdm_s = sim.RunTo(time_s, updateWind = True)

    # FDM step once, but no further than the next controller frame start
    if (fmuMode == 'Run'):
        tFdm_s = sim.RunTo(tSend_s + tFrameRate_s - sim.fdm.get_delta_t(), updateWind = True)

    # print(time_s, '\t', 1e3 * ((time.time() - tStart_s) - time_s), 1e3 * (time_s - tSend_s), '\t', 1e3 * (time_s - tReceive_s), '\t', 1e3 * (time_s - sim.fdm.get_sim_time()))
