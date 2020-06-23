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

pathRaptrsRoot = '/home/rega0051/Goldy3/RAPTRS_UMN'
pathRaptrsCommon = os.path.join(pathRaptrsRoot, 'software', 'src', 'common')

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
# SOC will connect to ptySimSoc : 

# Start a Virtual link for a psuedo-tty, SIL
#On Host: socat -d -d PTY,link=ptySimSoc,rawer PTY,link=ptySimFmu,rawer; stty sane;

# Start a Virtual link for a psuedo-tty through BBB USB, HIL
#On BBB: socat -d -d tcp-listen:8000,reuseaddr,fork PTY,link=ptySimSoc,rawer; stty sane;
#On Linux Host: socat -d -d PTY,link=ptySimFmu,rawer tcp:192.168.7.2:8000; stty sane;

if os.path.exists('ptySimFmu') == False:
    raise SystemExit('Error: "ptySimFmu" not found.')


# Visualization is defined for JSBSim in the OutputFgfs.xml, Flightgear should be running prior
# Linux: ./fgfs_JSBSim.sh UltraStick25e
# Windows: ./fgfs_JSBSim.bat UltraStick25e



# Once this script is running, and waiting...
# Linux: flight_amd64 thor.json


# SOC will request the FMU to Config Mode, then Run Mode

#Config:
#1) Read config messages
#
#Run:
#1) Read Sensors (JSBSim source, or otherwise faked)
#2) Send Sensor messages to SOC
#3) Read effector messages from SOC
#4) Send effector commands (JSBSim)
#5) Step Simulation (X times, for 20ms worth)
#6) wait until 20ms (from Sensor read)

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
fdm.load_ic('initGrnd.xml', True)

# Setup JSBSim to FlightGear
fdm.set_output_directive(path.join('scripts', 'OutputFgfs.xml'))

# Setup JSBSim Logging
fdm.set_output_directive(path.join('scripts', 'OutputLog.xml'))


# Display the Output
i = 0
while (str(fdm.get_output_filename(i),'utf-8') != ''):
    outStr = str(fdm.get_output_filename(i),'utf-8')
    if '/TCP' in outStr:
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
SocComms = AircraftSocComms(port = 'ptySimFmu')
fmuMode = 'Config'


# Config Messages
cfgMsgBasic = fmu_messages.config_basic()
cfgMsgMpu9250 = fmu_messages.config_mpu9250()
cfgMsgBme280 = fmu_messages.config_bme280()
cfgMsgUblox = fmu_messages.config_ublox()
cfgMsgAms5915 = fmu_messages.config_ams5915()
cfgMsgSwift = fmu_messages.config_swift()
cfgMsgAnalog = fmu_messages.config_analog()
cfgMsgEffector = fmu_messages.config_effector()
cfgMsgMission = fmu_messages.config_mission()
cfgMsgControlGain = fmu_messages.config_control_gain()

# Data Messages
dataMsgCommand = fmu_messages.command_effectors()
dataMsgCompound = fmu_messages.data_compound()
dataMsgBifrost = fmu_messages.data_bifrost()

AcquireTimeData = False
AcquireInternalMpu9250Data = False
AcquireInternalBme280Data = False


def add_msg(msgID, msgIndx, msgPayload):
    msgLen = len(msgPayload)
    
    msgData = b''
    msgData += msgID.to_bytes(1, byteorder = 'little')
    msgData += msgIndx.to_bytes(1, byteorder = 'little')
    msgData += msgLen.to_bytes(1, byteorder = 'little')
    msgData += msgPayload
    return msgData

#
tFrameRate_s = 1/50 # Desired Run rate
SocComms.Begin()
cfgMsgList = []
sensorList = []
effList = []

tStart_s = time.time()
tFdm_s = 0.0

while (True):
    tFrameStart_s = time.time()
        
    # Run Stuff
    # Send Bifrost Data...
    # FIXIT
    
    
    
    # Update Mission run mode
    # FIXIT
    
    #
    if (fmuMode is 'Run'):
        # Read all data from Sim, populate into the message
        # Send Data Messages to SOC
        # Loop through all items that have been configured
        dataMsg = b''

        if AcquireTimeData:
            msg = next((s for s in sensorList if s.id is fmu_messages.data_time().id), None)
            
            msg.time_us = int((tFrameStart_s - tStart_s) * 1e6)
            
            dataMsg += add_msg(msg.id, 0, msg.pack())
            
        if AcquireInternalMpu9250Data:
            msg = next((s for s in sensorList if s.id is fmu_messages.data_mpu9250().id), None)
            
            # FIXIT - Clipping in fmu_messages??
            accel_scale = 208.82724  # 1 / (9.807(g) * 16 / 32767.5) (+/-16g)
            gyro_scale = 938.71973  # 1 / (2000.0f/32767.5f * d2r (+/-2000deg/sec)
            mag_scale = 300  # fits range
            
            accel_lim = (2**15 - 1) / accel_scale
            gyro_lim = (2**15 -1) / gyro_scale
            mag_lim = (2**15 -1) / mag_scale

            msg.ReadStatus = 1
            msg.AccelX_mss = np.clip(fdm['sensor/imu/accelX_mps2'], -accel_lim, accel_lim)
            msg.AccelY_mss = np.clip(fdm['sensor/imu/accelY_mps2'], -accel_lim, accel_lim)
            msg.AccelZ_mss = np.clip(fdm['sensor/imu/accelZ_mps2'], -accel_lim, accel_lim)
            msg.GyroX_rads = np.clip(fdm['sensor/imu/gyroX_rps'], -gyro_lim, gyro_lim)
            msg.GyroY_rads = np.clip(fdm['sensor/imu/gyroY_rps'], -gyro_lim, gyro_lim)
            msg.GyroZ_rads = np.clip(fdm['sensor/imu/gyroZ_rps'], -gyro_lim, gyro_lim)
            msg.MagX_uT = np.clip(fdm['sensor/imu/magX_uT'], -mag_lim, mag_lim)
            msg.MagY_uT = np.clip(fdm['sensor/imu/magY_uT'], -mag_lim, mag_lim)
            msg.MagZ_uT = np.clip(fdm['sensor/imu/magZ_uT'], -mag_lim, mag_lim)
            msg.Temperature_C = 0.0 # FIXIT

            dataMsg += add_msg(msg.id, 0, msg.pack())
            
        if AcquireInternalBme280Data:
            msg = next((s for s in sensorList if s.id is fmu_messages.data_bme280().id), None)
            
            msg.ReadStatus = 1
            msg.Pressure_Pa = fdm['sensor/pitot/presStatic_Pa']
            msg.Temperature_C = fdm['sensor/pitot/temp_C']
            msg.Humidity_RH = 0.0
            
            dataMsg += add_msg(msg.id, 0, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_mpu9250_short().id]):
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in  enumerate([s for s in sensorList if s.id is fmu_messages.data_bme280().id]):
            if not (AcquireInternalBme280Data and indx == 1):
                dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_ublox().id]):
            
            import datetime
                
            def utctoweekseconds(utc = datetime.datetime.utcnow(), leapseconds = 37):
                """ Returns the GPS week, the GPS day, and the seconds 
                and microseconds since the beginning of the GPS week """
                datetimeformat = "%Y-%m-%d %H:%M:%S"
                epoch = datetime.datetime.strptime("1980-01-06 00:00:00", datetimeformat)

                tdiff = utc - epoch + datetime.timedelta(seconds = leapseconds)
                weeks = tdiff.days // 7
                gpsWeeks = weeks  % 1024 # Rollover at 1024 weeks since epoch
                gpsSec = tdiff.total_seconds() - 60*60*24*7 * weeks

                return gpsWeeks, gpsSec

            today = datetime.datetime.utcnow()
            gpsWeeks, gpsSec = utctoweekseconds(today)
            
            v_scale = 100
            v_lim = (2**15 - 1) / v_scale
            
            msg.Fix = 1
            msg.NumberSatellites = 0
            msg.TOW = int(round(gpsSec * 1000)) # UBLOX driver has TOW as miliseconds
            msg.Year = today.year # UTC
            msg.Month = today.month # UTC
            msg.Day = today.day # UTC
            msg.Hour = today.hour # UTC
            msg.Min = today.minute # UTC
            msg.Sec = today.second # UTC
            msg.Latitude_rad = fdm['sensor/gps/lat_rad']
            msg.Longitude_rad = fdm['sensor/gps/long_rad']
            msg.Altitude_m = fdm['sensor/gps/alt_m']
            msg.NorthVelocity_ms = np.clip(fdm['sensor/gps/vNorth_mps'], -v_lim, v_lim)
            msg.EastVelocity_ms = np.clip(fdm['sensor/gps/vEast_mps'], -v_lim, v_lim)
            msg.DownVelocity_ms = np.clip(fdm['sensor/gps/vDown_mps'], -v_lim, v_lim)
            msg.HorizontalAccuracy_m = 0.0
            msg.VerticalAccuracy_m = 0.0
            msg.VelocityAccuracy_ms = 0.0
            msg.pDOP = 0.0
            
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_swift().id]):

            msg.static_ReadStatus = 1
            msg.static_Pressure_Pa = fdm['sensor/pitot/presStatic_Pa']
            msg.static_Temperature_C = fdm['sensor/pitot/temp_C']
            msg.diff_ReadStatus = 1
            msg.diff_Pressure_Pa = fdm['sensor/pitot/presTip_Pa']
            msg.diff_Temperature_C = fdm['sensor/pitot/temp_C']
        
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_sbus().id]):
            
            # Read all the joystick values, populate the SBUS message
            pygame.event.get(pump = True) # Pump, retreive events so they clear
            msg.channels = JoyMap(joy)
            
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_ams5915().id]):
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_analog().id]):
            dataMsg += add_msg(msg.id, indx, msg.pack())
        
        # Send the Compound Sensor Message
        SocComms.SendMessage(dataMsgCompound.id, 0, dataMsg)
        
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

                #FIXIT - Map from effList to FlighControl.xml, use: fdm.query_property_catalog('_ext_')

                fdm['fcs/cmdMotor_ext_nd'] = dataMsgCommand.command[0]
                fdm['fcs/cmdElev_ext_rad'] = dataMsgCommand.command[1]
                fdm['fcs/cmdRud_ext_rad'] = dataMsgCommand.command[2]
                fdm['fcs/cmdAilR_ext_rad'] = dataMsgCommand.command[3]
                fdm['fcs/cmdFlapR_ext_rad'] = dataMsgCommand.command[4]
                fdm['fcs/cmdFlapL_ext_rad'] = dataMsgCommand.command[5]
                fdm['fcs/cmdAilL_ext_rad'] = dataMsgCommand.command[6]

#                print(dataMsgCommand.command)
            elif msgID == dataMsgBifrost.id:
                pass
                # dataMsgBifrost.unpack(msg = msgPayload)
            
            
        elif (fmuMode is 'Config'):
            print("Config Set: " + str(msgPayload))
            
            if msgID == cfgMsgBasic.id:
                cfgMsgBasic.unpack(msgPayload)
                
                sensorType = cfgMsgBasic.sensor
                cfgMsgList.append((msgID, sensorType))
                
                if sensorType == fmu_messages.sensor_type_time:
                    sensorList.append(fmu_messages.data_time())
                    AcquireTimeData = True
                    
                elif sensorType in [fmu_messages.sensor_type_input_voltage, fmu_messages.sensor_type_regulated_voltage, fmu_messages.sensor_type_pwm_voltage, fmu_messages.sensor_type_sbus_voltage]:
                    sensorList.append(fmu_messages.data_analog())
                    
                elif sensorType == fmu_messages.sensor_type_internal_bme280:
                    sensorList.append(fmu_messages.data_bme280())
                    AcquireInternalBme280Data = True
                    
                elif sensorType == fmu_messages.sensor_type_sbus:
                    sensorList.append(fmu_messages.data_sbus())
                    
                SocComms.SendAck(msgID)
                    
            elif msgID == cfgMsgMpu9250.id:
                cfgMsgList.append((msgID, 0))
                
                cfgMsgMpu9250.unpack(msgPayload)
                
                if cfgMsgMpu9250.internal == True:
                    AcquireInternalMpu9250Data = True
                    sensorList.append(fmu_messages.data_mpu9250())
                else:
                    sensorList.append(fmu_messages.data_mpu9250_short())
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgBme280.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_bme280())
                
                cfgMsgBme280.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgUblox.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_ublox())
                
                cfgMsgUblox.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgAms5915.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_ams5915())
                
                cfgMsgAms5915.unpack(msgPayload)
                cfgMsgAms5915.output
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgSwift.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_swift())
                
                cfgMsgSwift.unpack(msgPayload)
                cfgMsgSwift.output
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgAnalog.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_analog())
                
                cfgMsgAnalog.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgEffector.id:
                cfgMsgList.append((msgID, 0))
                
                cfgMsgEffector.unpack(msgPayload)
                effList.append(cfgMsgEffector.input)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgMission.id:
                cfgMsgList.append((msgID, 0))
                
                cfgMsgMission.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgControlGain.id:
                cfgMsgList.append((msgID, 0))
                
                cfgMsgControlGain.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            else:
                print("Unhandled message while in Configuration mode, id: " + str(msgID))

        # if 
    
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


