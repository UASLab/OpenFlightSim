#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

import os.path
import numpy as np
import AvlWrapper

#%% Path to AVL (output and script with all get generated and stored here)
avlPath = os.path.abspath(os.path.join('..', 'AeroDefinitions', 'AVL'))

# Configuration Setup
configName = 'UltraStick25e'

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


#%% Create and Run the Case Files
# Generate the caseList
caseList = AvlWrapper.GenCaseList(configName, caseVar, caseValList, trimList, trimListConst)

if 1:
    for case in caseList:
        # Create the script file for the run case
        print("Generating Case Script: %s" %case['name'])
        AvlWrapper.WriteCase(avlPath, case)

        print("Executing: %s" %case['name'])
        AvlWrapper.ExecuteCase(avlPath, case)


# Save the caseList
saveFile = os.path.join(avlPath, configName, 'caseList.npy')
np.save(saveFile, caseList)

