"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Louis Mueller, Chris Regan
"""

import os.path
import numpy as np

import JSBSimWriteXml as JSBXml

# Constants
ft2m = 0.3048
m2ft = 1.0/ft2m


#%% Aircraft Inputs
aircraftName = 'UltraStick25e'
aeroName = aircraftName + '_DegenGeom'

# JSB Output
saveJsbPath = os.path.join('..', 'Simulation')
saveJsbPath = os.path.abspath(os.path.join(saveJsbPath, 'aircraft', aircraftName))

#
load = {}

# Aero Data
load['Aero'] = {}
load['Aero']['aircraftName'] = aircraftName
load['Aero']['aeroName'] = aeroName

# VSP Dataset
load['Aero']['vspPath'] = os.path.abspath(os.path.join('..', 'AeroDefinitions', 'OpenVSP'))

# AVL DataSet
#load['Aero']['avlPath'] = os.path.abspath(os.path.join('..', 'AeroDefinitions', 'AVL', aircraftName))


# Load Aircraft oFdm data
import UltraStick25e
oFdm = UltraStick25e.LoadAircraftDef(load)

# FIXIT - Increase the base Drag
addedDrag = np.zeros_like(oFdm['Aero']['Coef']['CD']['zero'])
oFdm['Aero']['Coef']['CD']['total'] += addedDrag
oFdm['Aero']['Coef']['CD']['zero'] += addedDrag


#%%
import matplotlib.pyplot as plt

cond = 'beta_rad'
coef = 'dCMn'
dep = 'dElev_rad'

numB, numV, numA = oFdm['Aero']['Coef']['CL']['zero'].shape

xPlot = oFdm['Aero']['Cond'][cond][:,:,1]
yPlot = oFdm['Aero']['Coef'][coef][dep][:,:,1]

plt.plot(xPlot, yPlot, '-*')


#%% Prepare JSBSim-ML data (oFdm -> JSB)
# Define Conversion from oFdm to JSB-ML
convertFdm2Jsb = {}

# Surface Names, JSBSim names must match with Servo models and FCS system definition
convertFdm2Jsb['Surf'] = {}
convertFdm2Jsb['Surf']['oFdm'] = ['d' + s + '_rad' for s in oFdm['Aero']['surfNames']]
convertFdm2Jsb['Surf']['jsb'] = ['fcs/pos' + s + '_rad' for s in oFdm['Aero']['surfNames']]
convertFdm2Jsb['Surf']['scale'] = [None] * len(convertFdm2Jsb['Surf']['oFdm'])

# Aero Deriv dependencies definitions
convertFdm2Jsb['Dep'] = {}
convertFdm2Jsb['Dep']['oFdm'] = ['alpha_rad', 'beta_rad', 'dpHat_rps', 'dqHat_rps', 'drHat_rps'] + convertFdm2Jsb['Surf']['oFdm']
convertFdm2Jsb['Dep']['jsb'] = ['aero/alpha-rad', 'aero/beta-rad', 'velocities/p-aero-rad_sec', 'velocities/q-aero-rad_sec', 'velocities/r-aero-rad_sec'] + convertFdm2Jsb['Surf']['jsb']
convertFdm2Jsb['Dep']['scale'] = [None, None, 'aero/bi2vel', 'aero/ci2vel', 'aero/bi2vel'] + convertFdm2Jsb['Surf']['scale']

convertFdm2Jsb['Coef'] = {}
convertFdm2Jsb['Coef']['oFdm'] = ['zero']

# Aero Table defintions
convertFdm2Jsb['TableDef'] = {}
convertFdm2Jsb['TableDef']['jsb'] = ['aero/beta-deg', 'velocities/vt-fps', 'aero/alpha-deg']
convertFdm2Jsb['TableDef']['brkPts'] = [oFdm['Aero']['TableDef']['betaBrkPts_deg'], oFdm['Aero']['TableDef']['vBrkPts_mps'] * m2ft, oFdm['Aero']['TableDef']['alphaBrkPts_deg']]


#%% Create the XML
JSBXml.Aircraft(oFdm, convertFdm2Jsb, saveJsbPath, aircraftName)
