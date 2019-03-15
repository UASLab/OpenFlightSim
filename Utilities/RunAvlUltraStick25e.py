#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

import os
import numpy as np
import AvlWrapper

#%% Paths and files
dataPath = '../Aircraft/AVL Def'
dataPath = os.path.abspath(dataPath)

avlRun = './avl' # FIXIT - make an argument for different OSes


#%% Setup the case files
# Configuration Setup
aircraftName='UltraStick25e'

# Case Setup
caseVar='V'
caseValList = [12.0, 17.0, 23.0] # V does nothing at low speed if Alpha is also specified

# Trim Setup (Trimming with-out Alpha defined will yield trimmed alpha)
trimList = []
for alphaSet in np.linspace(-6, 14, 6): # Trim Alpha, 'with Alpha', to Value
  for betaSet in np.linspace(-10, 10, 5): # Trim Beta, 'with Beta', to Value
    trimList.append([['A A', alphaSet], ['B B', betaSet]])

# Trim Setup, Trim surfaces to zero, use pitch surface for 'PM' and rudder for "YM'
trimListConst = [['D1 D1',0.0],['D2 D2',0.0],['D3 D3',0.0],['D4 D4',0.0],['D5 PM',0.0],['D6 YM',0.0]]


#%% Setup the caseList definitions
caseList = []
for iCase, caseVal in enumerate(caseValList):
    for iTrim, trimSet in enumerate(trimList):
        caseName = "%s_%s%d" %(aircraftName, caseVar, caseVal)
        case = {
                'aircraftName': aircraftName,
                'name': "%s_%s%d" %(aircraftName, caseVar, caseVal),
                'cond': [["%s" %(caseVar), caseVal]],
                'trim': []
                }

        # Add the trimList
        for trim in trimSet:
            case['trim'].append(trim)
            name = trim[0].split(' ')[0]
            val = str(trim[1]).split('.')[0].replace('-','n')
            case['name'] += "_%s%s" %(name, val)

        # Add the trimListConst
        for trim in trimListConst:
            case['trim'].append(trim)

        caseList.append(case)


#%% Create and Run the Case Files
if 0:
    for case in caseList:
        # Create the script file for the run case
        print("Generating Case Script: %s" %case['name'])
        AvlWrapper.WriteCase(dataPath, case)

        print("Executing: %s" %case['name'])
        AvlWrapper.ExecuteCase(avlRun, dataPath, case)


# Save the caseList
saveFile = os.path.join(dataPath, aircraftName, 'caseList.npy')
np.save(saveFile, caseList)

