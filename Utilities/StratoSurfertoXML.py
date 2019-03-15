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
aircraftName = 'StratoSurfer'
aeroName = aircraftName + '_DegenGeom'

# VSP Input
loadPathVsp = '../Aircraft/OpenVSP Def/'
loadPathVsp = os.path.abspath(loadPathVsp)

# JSB Output
saveJsbPath = '../Sim'
saveJsbPath = os.path.abspath(os.path.join(saveJsbPath, 'aircraft', aircraftName))


#%% Parse VSP files and merge the data into a single set
vspData = VspParse.ParseAll(loadPathVsp, aircraftName, aeroName)


vspData['stab']['coef']['CD']['Base_Aero'] *= 1
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


#%% Aircraft Definition for oFdm
oFdm = {}

# Convert the VSP Aero data to oFdm
oFdm['aero'] = PyFdmUtilities.Vsp_to_Fdm(vspData, convertDef)
oFdm['aero']['surfNames'] = vspData['stab']['surfNames']


# Mass Properties - FIXIT - Need external MassProperty Source
# Mass Properties, mass has actual. Inertias are scaled from US25e test data.
oFdm['MassProp'] = {}

# Mass Properties
oFdm['MassProp']['mass_kg'] = 1.344

cgX = 0.3556
cgY = 0.0
cgZ = 0.0
oFdm['MassProp']['rCG_S_m'] = np.array([cgX, cgY, cgZ])

Ixx = 0.07151 * (oFdm['MassProp']['mass_kg'] / 1.959)
Iyy = 0.08636 * (oFdm['MassProp']['mass_kg'] / 1.959)
Izz = 0.15364 * (oFdm['MassProp']['mass_kg'] / 1.959)
Ixy = 0.0 * (oFdm['MassProp']['mass_kg'] / 1.959)
Ixz = 0.0 * (oFdm['MassProp']['mass_kg'] / 1.959)
Iyz = 0.0 * (oFdm['MassProp']['mass_kg'] / 1.959)
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
            [ 1.00000,-1.00000, 0.00000,-0.00000, 0.00000,-0.00000],
            [ 0.00000, 0.00000,-2.00000, 0.00000, 0.00000, 0.00000],
            [ 0.00000, 0.00000, 0.00000,-1.50000, 0.00000, 0.00000],
            [ 0.00000, 0.00000, 0.00000, 0.00000, 1.00000, 1.00000]
            ]

mixer['surfMix'] = np.linalg.pinv(mixer['surfEff'])
mixer['surfMix'][abs(mixer['surfMix'] / mixer['surfMix'].max()) < 0.05] = 0

oFdm['fcs']['Mixer'] = mixer


# Effector dynamic model, second-order with freeplay and limits
act = {}
for surf in oFdm['aero']['surfNames']:
    act[surf] = {}
    act[surf]['bandwidth_hz'] = 4.0 # Guess
    act[surf]['bandwidth_rps'] = act[surf]['bandwidth_hz'] * 2*np.pi
    act[surf]['lag_nd'] = round(200.0 / act[surf]['bandwidth_hz']) # Based on 200 Hz Sim Frame
    act[surf]['delay_s'] = 0.020 # Guess
    act[surf]['freeplay_rad'] = 1.0 * np.pi/180.0 # Guess
    act[surf]['min'] = -30.0 * np.pi/180.0
    act[surf]['max'] = 30.0 * np.pi/180.0

oFdm['act'] = act


# Create Propulsion data (motor and prop)
prop = {}
prop['nameMotor'] = 'Cobra2217'
prop['rMotor_S_m'] = np.array([21 * in2m, 0, 4.5 * in2m])
prop['sMotor_deg'] = np.array([0, -15, 0])

prop['nameProp'] = 'APC 6x4e'
prop['rProp_S_m'] = prop['rMotor_S_m']
prop['sProp_deg'] = prop['sMotor_deg']

prop['p_factor'] = 0.0
prop['sense'] = 1.0

oFdm['prop'] = {}
oFdm['prop']['Main'] = prop


# Create Sensor data
oFdm['Sensor'] = {}


#%% Create Gear data
mainH = 3.0 * in2m
mainY = 0.0 * in2m
mainX = 8.00 * in2m

tailX = 34.0 * in2m
tailH = 1.5 * in2m
tailY = 0.0 * in2m

wingH = -2.5 * in2m
wingY = 26.0 * in2m
wingX = 15.0 * in2m

cgX = oFdm['MassProp']['rCG_S_m'][0]

massMain = oFdm['MassProp']['mass_kg'] * (tailX - cgX) / -(mainX - tailX)
massTail = oFdm['MassProp']['mass_kg'] * (mainX - cgX) / -(tailX - cgX)
massWing = massTail # Needs some mass to compute the spring parameters, but should be 0.0


oFdm['gear'] = {}

# Belly skid
oFdm['gear']['Main'] = {}
oFdm['gear']['Main']['rGear_S_m'] = np.array([mainX, mainY, -mainH])
oFdm['gear']['Main']['FricStatic'] = 0.8
oFdm['gear']['Main']['FricDynamic'] = 0.5
oFdm['gear']['Main']['FricRoll'] = 0.25

wnDesire = 5.0 * 2*np.pi
dRatio = 1.0
oFdm['gear']['Main']['kSpring_Npm'] = wnDesire * wnDesire * massMain
oFdm['gear']['Main']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massMain


# Tail skid
oFdm['gear']['Tail'] = {}
oFdm['gear']['Tail']['rGear_S_m'] = np.array([tailX, tailY, -tailH])
oFdm['gear']['Tail']['FricStatic'] = 0.8
oFdm['gear']['Tail']['FricDynamic'] = 0.5
oFdm['gear']['Tail']['FricRoll'] = 0.25

wnDesire = 5.0 * 2*np.pi
dRatio = 1.0
oFdm['gear']['Tail']['kSpring_Npm'] = wnDesire * wnDesire * massTail
oFdm['gear']['Tail']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massTail


# Wing skid - left
oFdm['gear']['WingL'] = {}
oFdm['gear']['WingL']['rGear_S_m'] = np.array([wingX, -wingY, -wingH])
oFdm['gear']['WingL']['FricStatic'] = 0.8
oFdm['gear']['WingL']['FricDynamic'] = 0.5
oFdm['gear']['WingL']['FricRoll'] = 0.25

wnDesire = 5.0 * 2*np.pi
dRatio = 1.0
oFdm['gear']['WingL']['kSpring_Npm'] = wnDesire * wnDesire * massWing
oFdm['gear']['WingL']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massWing

# Wing skid - right
oFdm['gear']['WingR'] = copy.deepcopy(oFdm['gear']['WingL'])
oFdm['gear']['WingR']['rGear_S_m'][1] = -oFdm['gear']['WingL']['rGear_S_m'][1]



#%%
import matplotlib.pyplot as plt

cond = 'beta_rad'
coef = 'dCMn'
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


#%% Create the XML
JsbXml.Aircraft(oFdm, convertFdm2Jsb, saveJsbPath, aircraftName)
