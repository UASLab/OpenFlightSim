#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

import os
import numpy as np

import AvlWrapper
import PyFdmUtilities

import JsbSimWriteXml as JsbXml

#%% Paths and files
aircraftName='UltraStick25e'

dataPath = os.path.join('../Aircraft/AVL Def', aircraftName)
dataPath = os.path.abspath(dataPath)

# JSB Output
#saveJsbPath = '.'
saveJsbPath = '../Sim'
saveJsbPath = os.path.abspath(os.path.join(saveJsbPath, 'aircraft', aircraftName))


#%%
# Load the CaseList
loadCaseFile = os.path.join(dataPath, 'caseList.npy')
caseList = np.load(loadCaseFile)

## Parse and re-package the data
outPath = os.path.join(dataPath, 'out')
avlOutList = AvlWrapper.ParseOut(outPath, caseList)

avlData = AvlWrapper.Avl2Tables(avlOutList)


#%%
# Define some of the model specific conversions
convertDef = {}
convertDef['Lunit'] = 'in' # Specify the units used in the AVL model.

# Surface name converstions, AVL uses both the names and "d" surface syntax
convertDef['surf'] = {}
convertDef['surf']['Avl_D'] = ['d1','d2','d3','d4','d5','d6']
convertDef['surf']['names'] = ['FlapR','AilR','AilL','FlapL','Elev','Rud']
convertDef['surf']['Avl'] = ['dflapR','dailR','dailL','dflapL','delev','drud']
convertDef['surf']['fdm'] = ['d' + s + '_rad' for s in convertDef['surf']['names']]

in2m = 0.0254

#%%
oFdm = {}
oFdm['aero'] = PyFdmUtilities.Avl_to_Fdm(avlData, convertDef)
oFdm['aero']['surfNames'] = convertDef['surf']['names']


# Mass Properties - FIXIT - Need external MassProperty Source
oFdm['MassProp'] = {}

# Mass Properties per OpenFlight-master repository
oFdm['MassProp']['mass_kg'] = 1.959

cgX = 0.222
cgY = 0.0
cgZ = 0.046
oFdm['MassProp']['rCG_S_m'] = np.array([cgX, cgY, cgZ])

Ixx = 0.07151
Iyy = 0.08636
Izz = 0.15364
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
            [0.00000, -1.36545834, 1.36545834, 0.00000,0.00000, 0.00000],
            [0.00000, 0.00000, 0.00000, 0.00000,-2.33333068, 0.00000],
            [0.00000,0.00000, 0.00000, 0.00000, 0.00000,-1.43188557],
            [1.0, 0.00000, 0.00000, 1.0, 0.00000, 0.00000]
            ]

mixer['surfMix'] = np.linalg.pinv(mixer['surfEff'])
mixer['surfMix'][abs(mixer['surfMix'] / mixer['surfMix'].max()) < 0.05] = 0

oFdm['fcs']['Mixer'] = mixer


# Effector dynamic model, second-order with freeplay and limits
act = {}
for surf in oFdm['aero']['surfNames']:
    act[surf] = {}
    act[surf]['bandwidth_rps'] = 6.0 * 2*np.pi
    act[surf]['damp_nd'] = 0.8
    act[surf]['delay_s'] = 0.05
    act[surf]['freeplay_rad'] = 1.0 * np.pi/180.0
    act[surf]['min'] = -30.0 * np.pi/180.0
    act[surf]['max'] = 30.0 * np.pi/180.0

oFdm['act'] = act


# Create Propulsion data (motor and prop)
prop = {}
prop['nameMotor'] = 'Power25'
prop['rMotor_S_m'] = np.array([0, 0, 0])
prop['sMotor_deg'] = np.array([0, 0, 0])

prop['nameProp'] = 'APC 12x6e'
prop['rProp_S_m'] = np.array([-3 * in2m, 0, 0])
prop['sProp_deg'] = np.array([0, 0, 0])

prop['p_factor'] = 0.0

oFdm['prop'] = {}
oFdm['prop']['Main'] = prop


# Create Sensor data
oFdm['sens'] = {}


#%% Create Gear data
mainH = 7 * in2m
mainY = 6.5 * in2m
mainX = oFdm['MassProp']['rCG_S_m'][0] - 1.5 * in2m

tailH = 1.5 * in2m
tailX = 35 * in2m

oFdm['gear'] = {}
oFdm['gear']['Left'] = {}
oFdm['gear']['Left']['rGear_S_m'] = np.array([mainX, -mainY, -mainH])
oFdm['gear']['Left']['FricStatic'] = 0.8
oFdm['gear']['Left']['FricDynamic'] = 0.5
oFdm['gear']['Left']['FricRoll'] = 0.02
oFdm['gear']['Left']['kSpring_Npm'] = 1.35 * (oFdm['MassProp']['mass_kg'] * 9.81 / mainH)
oFdm['gear']['Left']['dampSpring_Nspm'] = 0.5 * oFdm['gear']['Left']['kSpring_Npm']

oFdm['gear']['Right'] = oFdm['gear']['Left']
oFdm['gear']['Right']['rGear_S_m'] = np.array([mainX, mainY, -mainH])

oFdm['gear']['Tail'] = oFdm['gear']['Left']
oFdm['gear']['Tail']['rGear_S_m'] = np.array([tailX, 0.0, -tailH])
oFdm['gear']['Tail']['kSpring_Npm'] = 0.5 * oFdm['gear']['Left']['kSpring_Npm']
oFdm['gear']['Tail']['dampSpring_Nspm'] = 0.5 * oFdm['gear']['Left']['dampSpring_Nspm']


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
convertFdm2Jsb['tableDef']['jsb'] = ['velocities/vt-fps', 'aero/alpha-deg', 'aero/beta-deg']
convertFdm2Jsb['tableDef']['brkPts'] = [oFdm['aero']['tableDef']['vBrkPts_mps'] * m2ft, oFdm['aero']['tableDef']['alphaBrkPts_deg'], oFdm['aero']['tableDef']['betaBrkPts_deg']]


#%% Create the XML
JsbXml.Aircraft(oFdm, convertFdm2Jsb, saveJsbPath, aircraftName)

