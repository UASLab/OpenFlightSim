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
aircraftName = 'UltraStick120'
aeroName = aircraftName + '_DegenGeom'

# VSP Input
loadPathVsp = '../Aircraft/OpenVSP Def/'
loadPathVsp = os.path.abspath(loadPathVsp)

# JSB Output
saveJsbPath = '../Sim'
saveJsbPath = os.path.abspath(os.path.join(saveJsbPath, 'aircraft', aircraftName))


#%% Parse VSP files and merge the data into a single set
vspData = VspParse.ParseAll(loadPathVsp, aircraftName, aeroName)


vspData['stab']['coef']['CD']['Base_Aero'] *= 2 # FIXIT - Increase the Drag, a lot!!
# vspData['stabTab'] is NOT a copy the values are linked!!

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


#%%
oFdm = {}

# Convert the VSP Aero data to oFdm
oFdm['aero'] = PyFdmUtilities.Vsp_to_Fdm(vspData, convertDef)
oFdm['aero']['surfNames'] = vspData['stab']['surfNames']



# Mass Properties - FIXIT - Need external MassProperty Source
oFdm['MassProp'] = {}

# Mass Properties per OpenFlight-master repository
oFdm['MassProp']['mass_kg'] = 7.411

cgX = 0.315
cgY = 0.0
cgZ = 0.072
oFdm['MassProp']['rCG_S_m'] = np.array([cgX, cgY, cgZ])

Ixx = 0.8568
Iyy = 1.0095
Izz = 1.7005
Ixy = 0
Ixz = 0.759258051/4
Iyz = 0
oFdm['MassProp']['inertia_kgm2'] = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])


# Flight Control System
oFdm['fcs'] = {}

# Pilot input scaling/sensitivity
pilot = {}
pilot['kRoll'] = 60.0 * np.pi / 180.0 # Normalized stick to cmdRoll
pilot['kPitch'] = -45.0 * np.pi / 180.0 # Normalized stick to cmdPitch
pilot['kYaw'] = -10.0 * np.pi / 180.0 # Normalized stick to cmdYaw
pilot['kFlap'] = 20.0 * np.pi / 180.0 # Normalized stick to cmdFlap

oFdm['fcs']['Pilot'] = pilot

# Mixer
mixer = {}
mixer['surfNames'] = oFdm['aero']['surfNames']
mixer['inputs'] = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdYaw_rps', 'cmdFlap_rad']
mixer['surfEff'] = [
            [-1.36545834, 1.36545834, 0.00000, 0.00000,0.00000, 0.00000],
            [0.00000, 0.00000, 0.00000, 0.00000, 0.00000,-2.33333068],
            [0.00000,0.00000,-1.43188557, 0.00000, 0.00000, 0.00000],
            [0.00000, 0.00000, 0.00000, 1.0, 1.0, 0.00000]
            ]

mixer['surfMix'] = np.linalg.pinv(mixer['surfEff'])
mixer['surfMix'][abs(mixer['surfMix'] / mixer['surfMix'].max()) < 0.05] = 0

oFdm['fcs']['Mixer'] = mixer


# Effector dynamic model, second-order with freeplay and limits
act = {}
for surf in oFdm['aero']['surfNames']:
    act[surf] = {}
    act[surf]['bandwidth_hz'] = 3.5
    act[surf]['bandwidth_rps'] = act[surf]['bandwidth_hz'] * 2*np.pi
    act[surf]['lag_nd'] = round(200.0 / act[surf]['bandwidth_hz']) # Based on 200 Hz Sim Frame
    act[surf]['delay_s'] = 0.020 # Guess
    act[surf]['freeplay_rad'] = 2.0 * np.pi/180.0
    act[surf]['min'] = -30.0 * np.pi/180.0
    act[surf]['max'] = 30.0 * np.pi/180.0

oFdm['act'] = act


# Create Propulsion data (motor and prop)
oFdm['prop'] = {}
oFdm['prop']['Motor'] = 'Power110'
oFdm['prop']['Prop'] = 'APC 16x8e'

# Create Propulsion data (motor and prop)
prop = {}
prop['nameMotor'] = 'Power110'
prop['rMotor_S_m'] = np.array([-4 * in2m, 0, 0])
prop['sMotor_deg'] = np.array([0, 0, 0])

prop['nameProp'] = 'APC 16x8e'
prop['rProp_S_m'] = np.array([-7 * in2m, 0, 0])
prop['sProp_deg'] = np.array([0, 0, 0])

prop['p_factor'] = 0.0
prop['sense'] = 1.0

oFdm['prop'] = {}
oFdm['prop']['Main'] = prop


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


## Airdata
oFdm['Sensor']['Pitot'] = {}

# Airdata Location and Orientation
oFdm['Sensor']['Pitot']['r_S_m'] = [0,0,0] # FIXIT - Not currently used
oFdm['Sensor']['Pitot']['s_deg'] = [0,0,0] # FIXIT - Not currently used

# Airdata Error Model
#   Pitot Vector - [presStatic_Pa, presTip_Pa, temp_C]
oFdm['Sensor']['Pitot']['delay_s'] = [0,0,0]
oFdm['Sensor']['Pitot']['lag'] = [0,0,0]
oFdm['Sensor']['Pitot']['noiseVar'] = [0,0,0]
oFdm['Sensor']['Pitot']['drift_ps'] = [0,0,0]
oFdm['Sensor']['Pitot']['gain_nd'] = [1,1,1]
oFdm['Sensor']['Pitot']['bias'] = [0,0,0]

# 5Hole
oFdm['Sensor']['5Hole'] = {}

# Airdata Location and Orientation
oFdm['Sensor']['5Hole']['r_S_m'] = [0,0,0] # FIXIT - Not currently used
oFdm['Sensor']['5Hole']['s_deg'] = [0,0,0] # FIXIT - Not currently used

#oFdm['Sensor']['5Hole']['alphaK1'] = [0.5*0.087, -0.5*0.087]
#oFdm['Sensor']['5Hole']['betaK1'] = [0.5*0.087, -0.5*0.087]
oFdm['Sensor']['5Hole']['alphaK2'] = 0.08
oFdm['Sensor']['5Hole']['betaK2'] = 0.08

# Airdata Error Model
#   5Hole [Method1] - [presStatic_Pa, presTip_Pa, presAlphaBot_Pa, presAlphaTop_Pa, presBetaRight_Pa, presBetaLeft_Pa, temp_C]
#   5Hole [Method2] - [presStatic_Pa, presTip_Pa, presAlpha_Pa, presBeta_Pa, temp_C]
oFdm['Sensor']['5Hole']['delay_s'] = [0,0,0,0,0]
oFdm['Sensor']['5Hole']['lag'] = [0,0,0,0,0]
oFdm['Sensor']['5Hole']['noiseVar'] = [0,0,0,0,0]
oFdm['Sensor']['5Hole']['drift_ps'] = [0,0,0,0,0]
oFdm['Sensor']['5Hole']['gain_nd'] = [1,1,1,1,1]
oFdm['Sensor']['5Hole']['bias'] = [0,0,0,0,0]


#%% Create Gear data
mainH = 12 * in2m
mainY = 12 * in2m
mainX = oFdm['MassProp']['rCG_S_m'][0] - 2.5 * in2m

tailH = 3.0 * in2m
tailX = 50 * in2m

cgX = oFdm['MassProp']['rCG_S_m'][0]

massMain = 0.5 * oFdm['MassProp']['mass_kg'] * (tailX - cgX) / -(mainX - tailX)
massTail = oFdm['MassProp']['mass_kg'] * (mainX - cgX) / -(tailX - cgX)


oFdm['gear'] = {}
oFdm['gear']['Left'] = {}
oFdm['gear']['Left']['rGear_S_m'] = np.array([mainX, -mainY, -mainH])
oFdm['gear']['Left']['FricStatic'] = 0.8
oFdm['gear']['Left']['FricDynamic'] = 0.5
oFdm['gear']['Left']['FricRoll'] = 0.02

wnDesire = 10 * 2*np.pi
dRatio = 1.0
oFdm['gear']['Left']['kSpring_Npm'] = wnDesire * wnDesire * massMain
oFdm['gear']['Left']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massMain

oFdm['gear']['Right'] = copy.deepcopy(oFdm['gear']['Left'])
oFdm['gear']['Right']['rGear_S_m'] = np.array([mainX, mainY, -mainH])


oFdm['gear']['Tail'] = copy.deepcopy(oFdm['gear']['Left'])
oFdm['gear']['Tail']['rGear_S_m'] = np.array([tailX, 0.0, -tailH])

wnDesire = 10 * 2*np.pi
dRatio = 1.0
oFdm['gear']['Tail']['kSpring_Npm'] = wnDesire * wnDesire * massTail
oFdm['gear']['Tail']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massTail

#%%
import matplotlib.pyplot as plt

cond = 'vTas_mps'
coef = 'dCMm'
dep = 'dElev_rad'

numB, numV, numA = oFdm['aero']['coef']['CL']['zero'].shape

xPlot = oFdm['aero']['cond'][cond][:,:,1]
yPlot = oFdm['aero']['coef'][coef][dep][:,:,1]

plt.plot(xPlot, yPlot, '-*')


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
        oFdm['aero']['coef'][coef][dep] = oFdm['aero']['coef'][coef][dep][2,1,:]

convertFdm2Jsb['tableDef']['jsb'] = convertFdm2Jsb['tableDef']['jsb'][2]
convertFdm2Jsb['tableDef']['brkPts'] = convertFdm2Jsb['tableDef']['brkPts'][2]

  #%% Create the XML
JsbXml.Aircraft(oFdm, convertFdm2Jsb, saveJsbPath, aircraftName)
