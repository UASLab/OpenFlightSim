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


#%% Argument Parsing
import argparse

# Initialize parser
parser = argparse.ArgumentParser()

# Adding optional argument
parser.add_argument("--modelName", default = 'UltraStick120', help = "Vehicle Model Name ('UltraStick25e', 'UltraStick120', 'F450')")
parser.add_argument("--runMode", default = 'SIL-TCP', help = "Simulation Run Mode ('SIL-PTY', 'SIL-TCP', 'PIL', 'HIL')")
#parser.add_argument("--VisForce", default = True, help = "Force FlightGear Visuals")

# Read arguments from command line
args = parser.parse_args()

# Local variables
modelName = args.modelName
runMode = args.runMode

#fgfsForce = False
#if args.VisForce:
#    fgfsForce = True


#%% Setup Connections
# Setup FMU-SOC comms
if runMode == 'SIL-PTY': # SIL Direct PTY-PTY
    host = None
    port = 'ptySimFmu'
    print('Running in SIL-PTY Mode')
elif runMode == 'SIL-TCP': # SIL via TCP on localhost
    host = 'localhost'
    port = 59600
    print('Running in SIL Mode: host = localhost')
elif runMode == ('PIL' or 'HIL'): # PIL/HIL via TCP
    # PIL
    host = '192.168.7.2'
    port = 59600
    print('Running in PIL/HIL Mode: host = 192.168.7.2')

# FlightGear Visuals
# Visualization is defined for JSBSim in the OutputFgfs.xml
# Linux: ./fgfs_JSBSim.sh {modelName}
# Windows: ./fgfs_JSBSim.bat {modelName}

import psutil
import subprocess

if psutil.LINUX:
    fgfsExc = 'fgfs'
    fgfsCmd = './fgfs_JSBSim.sh ' + modelName
elif psutil.WINDOWS:
    fgfsExc = 'fgfs.exe'
    fgfsCmd = './fgfs_JSBSim.bat ' + modelName
else:
    fgfsCmd = None

if fgfsExc in [p.name() for p in psutil.process_iter()]:
    fgfsRunning = True
else:
    fgfsRunning = False
#    if fgfsForce == True: # FGFS isn't running, force it.
#        subprocess.Popen([fgfsCmd], shell=True, stdin=None, stdout=None, stderr=None)


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
sim = JSBSimWrap(modelName)
sim.SetupIC('initGrnd.xml')
sim.SetupOutput()
sim.DispOutput()
sim.RunTrim()

#sim.SetTurb(turbType = 4, turbSeverity = 1, vWind20_mps = 3.0, vWindHeading_deg = 0.0)

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
                        matchList = difflib.get_close_matches(effName, sim.fdm.query_property_catalog('_ext_'))
                        if matchList != []:
                            match = matchList[0] # First is best

                            SocComms.effListFdm.append(match.strip(' (RW)'))
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
