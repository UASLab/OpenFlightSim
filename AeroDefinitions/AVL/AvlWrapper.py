#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

"""

import os
from subprocess import Popen

#%%
def WriteCase(dataPath, case):
    # The Case will be written such that the output files will be generated in the path that the avl executable resides.
    # This is to avoid a path length limitation that seems to exist within the avl code. The output files will then be
    # moved to the designated location.
    # Paths and files
    defPath = os.path.join(dataPath, case['configName'], 'def')
    outPath = os.path.join('temp')
    scriptPath = os.path.join(dataPath, case['configName'], 'script')

    # Retreive the case name
    caseName = case['name']

    # Make the defPath if required
    if not os.path.exists(defPath):
        os.makedirs(defPath)

    # Script file setup
    scriptFile = os.path.join(scriptPath, caseName + '.script')
    if not os.path.exists(scriptPath):
        os.makedirs(scriptPath)

    # Load file setup
    loadFile = os.path.join(defPath, case['configName'] + '.avl')

    # Mass file setup
    massFile = os.path.join(defPath, case['configName'] + '.mass')

    #%% Create the script file
    # Open the script file for writing
    fScript = open(scriptFile, 'w')

    # Load the configuration definition
    fScript.write('LOAD %s\n' %loadFile)

    # Load mass defintion
    fScript.write('MASS %s\n' %massFile)

    # Apply the mass configuration (setting to 0 applies to all)
    fScript.write('MSET 0\n')

    # Disable Plotting to speed the execution
    fScript.write('PLOP\n')
    fScript.write('G\n')
    fScript.write('\n')

    # Enter OPER menu
    fScript.write('OPER\n')

    # Define the case(s)
    fScript.write('C1\n') # Use C1 (level flight trim)
    fScript.write('N %s\n' %caseName)

    # Set the run conditions
    for cond in case['cond']:
        fScript.write('%s %4.3f\n' %(cond[0], cond[1]))

    fScript.write('\n')

    # Trim
    for trim in case['trim']:
        fScript.write('%s %4.3f\n' %(trim[0], trim[1]))

    # Execute the Run Case
    fScript.write('X\n')

    # Write the Run Case to a file
    fScript.write('%s\n' %'S')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.run'))

    # Write the Forces to a file
    #fScript.write('%s\n' %'W')
    #fScript.write('%s\n' %os.path.join(outPath,caseName + '.f'))

    # Write the Stability Derivatives to a file
    fScript.write('%s\n' %'ST')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.st'))

    # Write the Body Derivatives to a file
    fScript.write('%s\n' %'SB')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.sb'))

    # Write the Total Forces to a file
    #fScript.write('%s\n' %'FT')
    #fScript.write('%s\n' %os.path.join(outPath,caseName + '.ft'))

    # Write the Surface Forces to a file
    fScript.write('%s\n' %'FN')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.fn'))

    # Write the Strip Forces to a file
    fScript.write('%s\n' %'FS')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.fs'))

    # Write the Element Forces to a file
    fScript.write('%s\n' %'FE')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.fe'))

    # Write the Body Forces to a file
    fScript.write('%s\n' %'FB')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.fb'))

    # Write the Hinge Moments to a file
    fScript.write('%s\n' %'HM')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.hm'))

    # Write the Strip Shear, Moments to a file
    fScript.write('%s\n' %'VM')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.vm'))

    # Exit the OPER menu
    fScript.write('\n')

    # Enter MODE menu
    fScript.write('%s\n' %'MODE')
    fScript.write('%s\n' %'N')

    # Write the Mode Data to a file
    fScript.write('%s\n' %'W')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.eig'))

    # Write the System Output Data to a file
    fScript.write('%s\n' %'S')
    fScript.write('%s\n' %os.path.join(outPath,caseName + '.sys'))

    # Exit the MODE menu
    fScript.write('\n')
    fScript.write('\n')

    # Quit AVL
    fScript.write('Quit\n')

    #Close File
    fScript.close()

#%%
def ExecuteCase(avlRun, dataPath, case):

    ## Paths and files
    outPath = os.path.join(dataPath, case['configName'], 'out')
    scriptPath = os.path.join(dataPath, case['configName'], 'script')

    avlPath, avlExec = os.path.split(avlRun)
    avlPath = os.path.abspath(avlPath)
    tempPath = os.path.join(avlPath, 'temp')

    # Make the outPath if required
    if not os.path.exists(outPath):
        os.makedirs(outPath)

    # Make the tempPath if required
    if not os.path.exists(tempPath):
        os.makedirs(tempPath)

    caseName = case['name']
    scriptFile= os.path.join(scriptPath, caseName + '.script')
    logFile = scriptFile.replace('.script', '.log')

    # Clear the tempPath of files with the prefix caseName
    for outFile in os.listdir(tempPath):
        if caseName in outFile:
            os.remove(os.path.join(tempPath, outFile))

    ## Run the Script
    # Run the script by directing script into the avl executable and directing log out
    fScript = open(scriptFile, 'r')
    fLog = open(logFile, 'w')
    runProcess = Popen(avlRun, stdin=fScript, stdout=fLog)
    runProcess.wait()
    fLog.flush()
    fScript.close()

    status = runProcess.returncode

    # Move files from tempPath to outPath
    for outFile in os.listdir(tempPath):
        if caseName in outFile:
            if os.path.exists(os.path.join(outPath, outFile)):
                os.remove(os.path.join(outPath, outFile))
            os.rename(os.path.join(tempPath, outFile), os.path.join(outPath, outFile))

    return status


#%%
def ParseOut(outPath, caseList):

    # Parse these file types
    typeList = ['run','st','sb','hm']

    # Parse all the data into a single structure
    avlOutList = []
    for case in caseList:

        avlOut = {}

        caseName = case['name']
        print('Parsing: %s \n', caseName)

        for fileType in typeList:

            fileRead = os.path.join(outPath, caseName + '.' + fileType)

            if os.path.exists(fileRead):
                if 'run' in fileType:
                    avlOut['run'] = ParseDataEq(fileRead)

                elif 'st' in fileType:
                    avlOut['st'] =  ParseDataEq(fileRead)

                elif 'sb' in fileType:
                    avlOut['sb'] =  ParseDataEq(fileRead)

                elif 'hm' in fileType:
                    avlOut['hm'] =  ParseDataEq(fileRead)

                    for line in avlOut['hm']['raw']['lines'][6:]:
                        lineSplt = line.split('  ')
                        if len(lineSplt)>1:
                            varName = VarFix('Chinge' + lineSplt[0])
                            val = float(lineSplt[-1])
                            avlOut['hm'][varName] = val

            else:
                print('File does not exist, skipping: %s' %fileRead)

        avlOutList.append(avlOut)

    return avlOutList

#%%
# Little function to clean-up variable names and make them valid
def VarFix(varName):
    import re

    varName = varName.replace("'b/2V", 'Hat_Stab') # Example p'b/2v is pHat in the Stability Axis
    varName = varName.replace("'c/2V", 'Hat_Stab')
    varName = varName.replace("b/2V", 'Hat')
    varName = varName.replace("c/2V", 'Hat')
    varName = varName.replace("'tot", 'tot_Stab')

    varName = re.sub('\W|^(?=\d)','_', varName) # fix any non-valid variable characters in the name

    return varName

#%% Find and Parse 'key = value' pairs from a string of text
def KeyEqVal(line):
    import re

    reKey = '\s*(?:\S+)\s*'
    reValue = '\s*(?:\S+)\s*'
    reEq = reKey + '=' + reValue

    retList = []
    eqMatches = re.findall(reEq, line)
    for eqMatch in eqMatches:
        eqMatch = eqMatch.replace(' ', '')
        eqSplit = eqMatch.split('=')

        varName = VarFix(eqSplit[0])
        val = float(eqSplit[1].replace(')',''))

        retList.append((eqMatch, varName, val)) # Return a list of tuples

    return retList


#%% Parse
def ParseDataEq(fileRead):
    #%% Open File
    file = open(fileRead, "r")

    # Store the raw lines into the dictionary
    avlData = {}
    avlData['raw'] = {}
    avlData['raw']['lines'] = [line.rstrip('\n') for line in file]

    file.close()


    #%%
    # Fix the variable: "Clb Cnr / Clr Cnb"
    for line in avlData['raw']['lines']: line.replace('Clb Cnr / Clr Cnb', 'Clb_Cnr__Clr_Cnb')

    #%% Parse each line for multiple key=value pairs, assign into dictionary
    avlData['raw']['eqList'] = [] # Retain the strings, just in case!
    for line in avlData['raw']['lines']:
        tempList = KeyEqVal(line)
        for temp in tempList:
            avlData['raw']['eqList'].append(temp[0])
            avlData[VarFix(temp[1])] = temp[2]

    #%%
    return avlData


#%% Transform the Flat avl Data into table form
def Avl2Tables(avlOutList):
    import numpy as np

    avlTable = {}

    # Leverage the fact that avl cases are run varying alpha, then mach, then beta
    avlTable['tableDef'] = {}
    avlTable['tableDef']['brkPtVars'] = ['velocity', 'alpha', 'beta']
    avlBrkNames = ['velBrkPts', 'alphaBrkPts', 'betaBrkPts']

    # Breakpoints
    avlTable['tableDef']['breakIndex'] = []

    for brkPtVar in avlTable['tableDef']['brkPtVars']:
        avlTable['tableDef'][brkPtVar] = []

    for avlOut in avlOutList:
        brkList = []
        for brkPtVar in avlTable['tableDef']['brkPtVars']:
            brkList.append(avlOut['run'][brkPtVar])
            avlTable['tableDef'][brkPtVar].append(avlOut['run'][brkPtVar])
        avlTable['tableDef']['breakIndex'].append(tuple(brkList))

    shapeTable = []
    for iBrk, brkPtVar in enumerate(avlTable['tableDef']['brkPtVars']):
        avlTable['tableDef'][avlBrkNames[iBrk]] = np.unique(avlTable['tableDef'][brkPtVar])
        shapeTable.append(len(avlTable['tableDef'][avlBrkNames[iBrk]]))

    # Create the Condition Tables from the 'run' file
    avlTable['run'] = {}
    for cond in avlOutList[0]['run'].keys():
        if cond in 'raw':
            continue
        avlTable['run'][cond] = []
        for avlOut in avlOutList:
            avlTable['run'][cond].append(avlOut['run'][cond])

        avlTable['run'][cond] = np.array(avlTable['run'][cond]).reshape(shapeTable)

    # Create the Stability Tables
    avlTable['st'] = {}
    for coef in avlOutList[0]['st'].keys():
        if coef in ['raw', 'Surfaces', 'Strips', 'Vortices'] or coef in avlTable['run'].keys():
            continue
        avlTable['st'][coef] = []
        for avlOut in avlOutList:
            avlTable['st'][coef].append(avlOut['st'][coef])

        avlTable['st'][coef] = np.array(avlTable['st'][coef]).reshape(shapeTable)


    # Create the Stability-Body Tables
    avlTable['sb'] = {}
    for coef in avlOutList[0]['sb'].keys():
        if coef in ['raw', 'Surfaces', 'Strips', 'Vortices'] or coef in avlTable['run'].keys():
            continue
        avlTable['sb'][coef] = []
        for avlOut in avlOutList:
            avlTable['sb'][coef].append(avlOut['sb'][coef])

        avlTable['sb'][coef] = np.array(avlTable['sb'][coef]).reshape(shapeTable)

    # Create the Hinge Moment Tables
    avlTable['hm'] = {}
    for coef in avlOutList[0]['hm'].keys():
        if coef in ['raw'] or coef in avlTable['run'].keys():
            continue
        avlTable['hm'][coef] = []
        for avlOut in avlOutList:
            avlTable['hm'][coef].append(avlOut['hm'][coef])

        avlTable['hm'][coef] = np.array(avlTable['hm'][coef]).reshape(shapeTable)


    return (avlTable)


