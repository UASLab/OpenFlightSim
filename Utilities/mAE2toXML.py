#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Louis Mueller
University of Minnesota UAV Lab

General Steps
1. Load and Parse VSP data into a Dict
2. Convert and Transform data into native format
3. Prepare JSB setup, write XML for JSB

"""
import os.path
import numpy as np
import copy

import VspParse
import PyFdmUtilities

import JsbSimWriteXml as JsbXml


#%% Aircraft Inputs
aircraftName = 'mAEWing2'
aeroName = aircraftName + '_DegenGeom'

# VSP Input
loadPathVsp = '../Aircraft/OpenVSP Def/'
loadPathVsp = os.path.abspath(loadPathVsp)

# JSB Output
saveJsbPath = '../Sim'
saveJsbPath = os.path.abspath(os.path.join(saveJsbPath, 'aircraft', aircraftName))


#%% Parse VSP files and merge the data into a single set
vspData = VspParse.ParseAll(loadPathVsp, aircraftName, aeroName)


vspData['stab']['coef']['CD']['Base_Aero'] *= 1.5 # FIXIT - Increase the Drag
# vspData['stabTab'] is NOT a copy, the values are referenced!!


#%%
# Define some of the model specific conversions
convertDef = {}
convertDef['Lunit'] = 'm' # Specify the units used in the VSP model.

# Surface name converstions
convertDef['surf'] = {}
convertDef['surf']['names'] = vspData['stab']['surfNames']
convertDef['surf']['Vsp'] = ['d' + s + '_rad' for s in convertDef['surf']['names']]
convertDef['surf']['Vsp_Grp'] = ['ConGrp_' + str(d) for d in range(1, 1+len(convertDef['surf']['Vsp']))]
convertDef['surf']['fdm'] = ['d' + s + '_rad' for s in convertDef['surf']['names']]

in2m = 0.0254


#%% Aircraft Definition for oFdm
oFdm = {}

# Convert the VSP Aero data to oFdm
oFdm['aero'] = PyFdmUtilities.Vsp_to_Fdm(vspData, convertDef)
oFdm['aero']['surfNames'] = vspData['stab']['surfNames']


# Mass Properties - FIXIT - Need external MassProperty Source
oFdm['MassProp'] = {}

# Mass Properties per OpenFlight-master repository
oFdm['MassProp']['mass_kg'] = 17.444

cgX = 0.7811
cgY = 0.0000
cgZ = 0.0687
oFdm['MassProp']['rCG_S_m'] = np.array([cgX, cgY, cgZ])

Ixx = 10.943
Iyy =  1.418
Izz = 13.300
Ixy = 0.0
Ixz = 0.0
Iyz = 0.0
oFdm['MassProp']['inertia_kgm2'] = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])


# Flight Control System
oFdm['fcs'] = {}

# Pilot input scaling/sensitivity
pilot = {}
pilot['kRoll'] = 60 * np.pi / 180.0 # Normalized stick to cmdRoll
pilot['kPitch'] = -20 * np.pi / 180.0 # Normalized stick to cmdPitch
pilot['kYaw'] = -20 * np.pi / 180.0 # Normalized stick to cmdYaw
pilot['kFlap'] = 20 * np.pi / 180.0 # Normalized stick to cmdFlap

oFdm['fcs']['Pilot'] = pilot

# Mixer
mixer = {}
mixer['surfNames'] = oFdm['aero']['surfNames']
mixer['inputs'] = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdYaw_rps', 'cmdFlap_rad']
mixer['surfEff'] = [
            [0,0,-0.12566766209044647,0.12566766209044647,-0.17400791474534627,0.17400791474534627,-0.2107643429473072,0.2107643429473072,-0.21455703129167966,0.21455703129167966,0,0],
            [0,0,0,0,-0.24059736290652464,-0.24059736290652464,-0.37964759401991566,-0.37964759401991566,-0.45864997674431435,-0.45864997674431435,0,0],
            [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
            [1.0,1.0,0.86466898,0.86466898,0.66642794,0.66642794,0.0,0.0,-0.61500559,-0.61500559,0.28647549,0.28647549]
            ]

mixer['surfMix'] = np.linalg.pinv(mixer['surfEff'])
mixer['surfMix'][abs(mixer['surfMix'] / mixer['surfMix'].max()) < 0.05] = 0

oFdm['fcs']['Mixer'] = mixer


# Effector dynamic model, second-order with freeplay and limits
act = {}
for surf in oFdm['aero']['surfNames']:
    act[surf] = {}
    act[surf]['bandwidth_hz'] = 25.0
    act[surf]['bandwidth_rps'] = act[surf]['bandwidth_hz'] * 2*np.pi
    act[surf]['lag_nd'] = round(200.0 / act[surf]['bandwidth_hz']) # Based on 200 Hz Sim Frame
    act[surf]['delay_s'] = 0.007 # Measured 0.007 sec
    act[surf]['freeplay_rad'] = 0.25 * np.pi/180.0
    act[surf]['min'] = -20.0 * np.pi/180.0
    act[surf]['max'] = 20.0 * np.pi/180.0

oFdm['act'] = act


# Create Propulsion data (motor and prop)
propL = {}
propL['nameMotor'] = 'Typhoon_HET_700-68_1200Kv'
propL['rMotor_S_m'] = np.array([33 * in2m, -5 * in2m, 5 * in2m])
propL['sMotor_deg'] = np.array([0, 0, 0])

propL['nameProp'] = 'Jetfan90v2'
propL['rProp_S_m'] = propL['rMotor_S_m']
propL['sProp_deg'] = propL['sMotor_deg']

propL['p_factor'] = 0.0
propL['sense'] = 1.0

propR = copy.deepcopy(propL)
propR['rMotor_S_m'][1] = -propR['rMotor_S_m'][1]
propR['rProp_S_m'] = propR['rMotor_S_m']

oFdm['prop'] = {}
oFdm['prop']['Left'] = propL
oFdm['prop']['Right'] = propR


#%% Create Sensor data
oFdm['Sensor'] = {}

## IMU
oFdm['Sensor']['Imu'] = {}

# Accel Location and Orientation
oFdm['Sensor']['Imu']['Accel'] = {}
oFdm['Sensor']['Imu']['Accel']['r_S_m'] = [0,0,0]
oFdm['Sensor']['Imu']['Accel']['s_deg'] = [0,0,0]

# Accel Error Model Parameters (units are _mps2)
oFdm['Sensor']['Imu']['Accel']['delay_s'] = [0,0,0]
oFdm['Sensor']['Imu']['Accel']['lag'] = [0,0,0]
oFdm['Sensor']['Imu']['Accel']['noiseVar'] = [0,0,0]
oFdm['Sensor']['Imu']['Accel']['drift_ps'] = [0,0,0]
oFdm['Sensor']['Imu']['Accel']['gain_nd'] = [1,1,1]
oFdm['Sensor']['Imu']['Accel']['bias'] = [0,0,0]

# Gyro Location and Orientation
oFdm['Sensor']['Imu']['Gyro'] = {}
oFdm['Sensor']['Imu']['Gyro']['r_S_m'] = oFdm['Sensor']['Imu']['Accel']['r_S_m']
oFdm['Sensor']['Imu']['Gyro']['s_deg'] = oFdm['Sensor']['Imu']['Accel']['s_deg']

# Gyro Error Model Parameters (units are _rps)
oFdm['Sensor']['Imu']['Gyro']['delay_s'] = oFdm['Sensor']['Imu']['Accel']['delay_s']
oFdm['Sensor']['Imu']['Gyro']['lag'] = oFdm['Sensor']['Imu']['Accel']['lag']
oFdm['Sensor']['Imu']['Gyro']['noiseVar'] = [0,0,0]
oFdm['Sensor']['Imu']['Gyro']['drift_ps'] = [0,0,0]
oFdm['Sensor']['Imu']['Gyro']['gain_nd'] = [1,1,1]
oFdm['Sensor']['Imu']['Gyro']['bias'] = [0,0,0]

# Magnetometer Location and Orientation
oFdm['Sensor']['Imu']['Mag'] = {}
oFdm['Sensor']['Imu']['Mag']['r_S_m'] = oFdm['Sensor']['Imu']['Accel']['r_S_m']
oFdm['Sensor']['Imu']['Mag']['s_deg'] = oFdm['Sensor']['Imu']['Accel']['s_deg']

# Magnetometer Error Model Parameters (units are _nT)
oFdm['Sensor']['Imu']['Mag']['delay_s'] = oFdm['Sensor']['Imu']['Accel']['delay_s']
oFdm['Sensor']['Imu']['Mag']['lag'] = oFdm['Sensor']['Imu']['Accel']['lag']
oFdm['Sensor']['Imu']['Mag']['noiseVar'] = [0,0,0]
oFdm['Sensor']['Imu']['Mag']['drift_ps'] = [0,0,0]
oFdm['Sensor']['Imu']['Mag']['gain_nd'] = [1,1,1]
oFdm['Sensor']['Imu']['Mag']['bias'] = [0,0,0]


## GPS
oFdm['Sensor']['Gps'] = {}

# Gps Location
oFdm['Sensor']['Gps']['r_S_m'] = [0,0,0] # FIXIT - Not currently used

# GPS Position Error Model # NOTE units are radians, radians, meters for Lat and Long!!
oFdm['Sensor']['Gps']['Pos'] = {}
oFdm['Sensor']['Gps']['Pos']['delay_s'] = [0,0,0]
oFdm['Sensor']['Gps']['Pos']['lag'] = [0,0,0]
oFdm['Sensor']['Gps']['Pos']['noiseVar'] = [0,0,0]
oFdm['Sensor']['Gps']['Pos']['drift_ps'] = [0,0,0]
oFdm['Sensor']['Gps']['Pos']['gain_nd'] = [1,1,1]
oFdm['Sensor']['Gps']['Pos']['bias'] = [0,0,0]

# GPS Velocity Error Model
oFdm['Sensor']['Gps']['Vel'] = {}
oFdm['Sensor']['Gps']['Vel']['delay_s'] = [0,0,0]
oFdm['Sensor']['Gps']['Vel']['lag'] = [0,0,0]
oFdm['Sensor']['Gps']['Vel']['noiseVar'] = [0,0,0]
oFdm['Sensor']['Gps']['Vel']['drift_ps'] = [0,0,0]
oFdm['Sensor']['Gps']['Vel']['gain_nd'] = [1,1,1]
oFdm['Sensor']['Gps']['Vel']['bias'] = [0,0,0]


## Airdata - 5Hole
oFdm['Sensor']['5Hole'] = {}

# Airdata Location and Orientation
oFdm['Sensor']['5Hole']['r_S_m'] = [0,0,0] # FIXIT - Not currently used
oFdm['Sensor']['5Hole']['s_deg'] = [0,0,0] # FIXIT - Not currently used

oFdm['Sensor']['5Hole']['alphaK1'] = [0.5*0.087, -0.5*0.087]
oFdm['Sensor']['5Hole']['betaK1'] = [0.5*0.087, -0.5*0.087]
#oFdm['Sensor']['5Hole']['alphaK2'] = 0.08
#oFdm['Sensor']['5Hole']['betaK2'] = 0.08

# Airdata Error Model
#   5Hole [Method1] - [presStatic_Pa, presTip_Pa, presAlphaBot_Pa, presAlphaTop_Pa, presBetaRight_Pa, presBetaLeft_Pa, temp_C]
#   5Hole [Method2] - [presStatic_Pa, presTip_Pa, presAlpha_Pa, presBeta_Pa, temp_C]
oFdm['Sensor']['5Hole']['delay_s'] = [0,0,0,0,0,0,0]
oFdm['Sensor']['5Hole']['lag'] = [0,0,0,0,0,0,0]
oFdm['Sensor']['5Hole']['noiseVar'] = [0,0,0,0,0,0,0]
oFdm['Sensor']['5Hole']['drift_ps'] = [0,0,0,0,0,0,0]
oFdm['Sensor']['5Hole']['gain_nd'] = [1,1,1,1,1,1,1]
oFdm['Sensor']['5Hole']['bias'] = [0,0,0,0,0,0,0]




#%% Create Gear data
mainH = 10.0 * in2m
mainY = 0.0 * in2m
mainX = 32.45 * in2m #mainX = oFdm['MassProp']['rCG_S_m'][0] + 1.7 * in2m

noseX = 12.0 * in2m
noseH = mainH + np.tan(3.0 * np.pi / 180.0) * (mainX - noseX) # Achieve ~2 deg inclination
noseY = 0.0 * in2m

wingH = 0.0 * in2m # Should be ~0.0, but it trims onto the wing
wingY = 84.0 * in2m
wingX = 48.0 * in2m

cgX = oFdm['MassProp']['rCG_S_m'][0]

massMain = oFdm['MassProp']['mass_kg'] * (noseX - cgX) / -(mainX - noseX)
massNose = oFdm['MassProp']['mass_kg'] * (mainX - cgX) / -(noseX - cgX)
massWing = massNose # Needs some mass, but should be 0.0


oFdm['gear'] = {}

# Main
oFdm['gear']['Main'] = {}
oFdm['gear']['Main']['rGear_S_m'] = np.array([mainX, mainY, -mainH])
oFdm['gear']['Main']['FricStatic'] = 0.8
oFdm['gear']['Main']['FricDynamic'] = 0.5
oFdm['gear']['Main']['FricRoll'] = 0.15

wnDesire = 5.0 * 2*np.pi
dRatio = 1.0
oFdm['gear']['Main']['kSpring_Npm'] = wnDesire * wnDesire * massMain
oFdm['gear']['Main']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massMain


# Nose skid
oFdm['gear']['Nose'] = {}
oFdm['gear']['Nose']['rGear_S_m'] = np.array([noseX, noseY, -noseH])
oFdm['gear']['Nose']['FricStatic'] = 0.8
oFdm['gear']['Nose']['FricDynamic'] = 0.5
oFdm['gear']['Nose']['FricRoll'] = 0.25

wnDesire = 5.0 * 2*np.pi
dRatio = 1.0
oFdm['gear']['Nose']['kSpring_Npm'] = wnDesire * wnDesire * massNose
oFdm['gear']['Nose']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massNose


# Wing skid - left
oFdm['gear']['WingL'] = {}
oFdm['gear']['WingL']['rGear_S_m'] = np.array([wingX, -wingY, -wingH])
oFdm['gear']['WingL']['FricStatic'] = 0.8
oFdm['gear']['WingL']['FricDynamic'] = 0.5
oFdm['gear']['WingL']['FricRoll'] = 0.5

wnDesire = 5.0 * 2*np.pi
dRatio = 1.0
oFdm['gear']['WingL']['kSpring_Npm'] = wnDesire * wnDesire * massWing
oFdm['gear']['WingL']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massWing

# Wing skid - right
oFdm['gear']['WingR'] = copy.deepcopy(oFdm['gear']['WingL'])
oFdm['gear']['WingR']['rGear_S_m'][1] = -oFdm['gear']['WingL']['rGear_S_m'][1]


#%% Winch
oFdm['Winch'] = {}
oFdm['Winch']['rHook_S_m'] = np.array([0.1, 0.0, -0.1])
oFdm['Winch']['sHook_deg'] = np.array([0.0, 0.0, 0.0])


#%%
import matplotlib.pyplot as plt

cond = 'vTas_mps'
coef = 'dCMm'
dep = 'alpha_rad'

numB, numV, numA = oFdm['aero']['coef']['CL']['zero'].shape

xPlot = oFdm['aero']['cond'][cond][2,:,(0,1,2,4)]
yPlot = oFdm['aero']['coef'][coef][dep][2,:,(0,1,2,4)]

plt.plot(xPlot.transpose(), yPlot.transpose(), '-*')


#%% Prepare JSBSim-ML data (oFdm -> JSB)
m2ft = 1.0/0.3049

# Define Conversion from oFdm to JSB-ML
convertFdm2Jsb = {}

# Surface Names, JSBSim names must match with Servo models and FCS system definition
convertFdm2Jsb['surf'] = {}
convertFdm2Jsb['surf']['fdm'] = ['d' + s + '_rad' for s in oFdm['aero']['surfNames']]
convertFdm2Jsb['surf']['jsb'] = ['fcs/pos' + s + '_rad' for s in oFdm['aero']['surfNames']]
convertFdm2Jsb['surf']['scale'] = [None] * len(convertFdm2Jsb['surf']['fdm'])

# Aero Deriv dependencies definitions
convertFdm2Jsb['dep'] = {}
convertFdm2Jsb['dep']['fdm'] = ['alpha_rad', 'beta_rad', 'dpHat_rps', 'dqHat_rps', 'drHat_rps'] + convertFdm2Jsb['surf']['fdm']
convertFdm2Jsb['dep']['jsb'] = ['aero/alpha-rad', 'aero/beta-rad', 'velocities/p-aero-rad_sec', 'velocities/q-aero-rad_sec', 'velocities/r-aero-rad_sec'] + convertFdm2Jsb['surf']['jsb']
convertFdm2Jsb['dep']['scale'] = [None, None, 'aero/bi2vel', 'aero/ci2vel', 'aero/bi2vel'] + convertFdm2Jsb['surf']['scale']

convertFdm2Jsb['coef'] = {}
convertFdm2Jsb['coef']['fdm'] = ['zero']

# Aero Table defintions
convertFdm2Jsb['tableDef'] = {}
convertFdm2Jsb['tableDef']['jsb'] = ['aero/beta-deg', 'velocities/vt-fps', 'aero/alpha-deg']
convertFdm2Jsb['tableDef']['brkPts'] = [oFdm['aero']['tableDef']['betaBrkPts_deg'], oFdm['aero']['tableDef']['vBrkPts_mps'] * m2ft, oFdm['aero']['tableDef']['alphaBrkPts_deg']]


#%% Reduce the Tables to just use Alpha
for coef in oFdm['aero']['coef'].keys():
    for dep in oFdm['aero']['coef'][coef].keys():
        oFdm['aero']['coef'][coef][dep] = oFdm['aero']['coef'][coef][dep][2,:,(0,1,2,4)]

convertFdm2Jsb['tableDef']['jsb'] = convertFdm2Jsb['tableDef']['jsb'][1:]
convertFdm2Jsb['tableDef']['brkPts'] = convertFdm2Jsb['tableDef']['brkPts'][1:]
convertFdm2Jsb['tableDef']['brkPts'][1] = convertFdm2Jsb['tableDef']['brkPts'][1][([0,1,2,4])]


#%% Create the XML
JsbXml.Aircraft(oFdm, convertFdm2Jsb, saveJsbPath, aircraftName)
