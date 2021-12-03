"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota.
See: LICENSE.md for complete license details.

Author: Louis Mueller, Chris Regan
"""

import os.path
from xml.etree import ElementTree as ET

import numpy as np


ft2m = 0.3048
psf2pa = 47.88026

#%% Save the XML in pretty-ish print
def SaveXml(elem, saveFile):
    from xml.dom import minidom

    uglyXml = ET.tostring(elem, 'utf-8')
    prettyXml = minidom.parseString(uglyXml).toprettyxml(indent='    ', newl = '\r\n')

    os.makedirs(os.path.dirname(saveFile), exist_ok=True)
    with open(saveFile, 'w') as saveXML:
        saveXML.write(prettyXml)

    saveXML.close()

#%% Function

def Aircraft(oFdm, convertFdm2Jsb, saveJsbPath, aircraftName):


    # Start JSB-ML with etree
    elemAircraft = ET.Element('fdm_config', version = '2.0', release = 'Alpha')


    # Create the Pilot input as a seperate XML file, direct the Aircraft definition to use
    fcsFile = 'FlightControl.xml'
    ET.SubElement(elemAircraft, 'flight_control', file = fcsFile)

    SaveXml(FlightControl(oFdm), os.path.join(saveJsbPath, fcsFile))


    # Effectors as a seperate XML file, direct the Aircraft definition to use
    effFile = 'Effectors.xml'
    ET.SubElement(elemAircraft, 'system', file = effFile)

    SaveXml(Effectors(oFdm), os.path.join(saveJsbPath, effFile))


    # Create the Mass Properties input as a seperate XML file, direct the Aircraft definition to use
    massFile = 'Mass.xml'
    ET.SubElement(elemAircraft, 'mass_balance', file = massFile)

    SaveXml(MassBalance(oFdm), os.path.join(saveJsbPath, massFile))


    # Create the Gear input as a seperate XML file, direct the Aircraft definition to use
    gearFile = 'Gear.xml'
    ET.SubElement(elemAircraft, 'ground_reactions', file = gearFile)

    SaveXml(GroundReactions(oFdm), os.path.join(saveJsbPath, gearFile))


    # Create the Propulsion input as a seperate XML file, direct the Aircraft definition to use
    propFile = 'Propulsion.xml'
    ET.SubElement(elemAircraft, 'propulsion', file = propFile)

    SaveXml(Propulsion(oFdm), os.path.join(saveJsbPath, propFile))


    # Metrics and Aerodynamics as a seperate XML file, direct the Aircraft definition to use
    # Group the Metrics and Aero by similar naming; the dimensionalization inherent to Aero is provided by the Metrics
    metricsFile = 'Metrics.xml'
    ET.SubElement(elemAircraft, 'metrics', file = metricsFile)

    SaveXml(Metrics(oFdm), os.path.join(saveJsbPath, metricsFile))


    aeroFile = 'Aero.xml'
    ET.SubElement(elemAircraft, 'aerodynamics', file = aeroFile)

    SaveXml(Aerodynamics(oFdm, convertFdm2Jsb), os.path.join(saveJsbPath, aeroFile))


    # Launcher as a seperate XML file, direct the Aircraft definition to use
    if 'Winch' in oFdm.keys() :
        winchFile = 'Winch.xml'
        ET.SubElement(elemAircraft, 'external_reactions', file = winchFile)

        SaveXml(Winch(oFdm), os.path.join(saveJsbPath, winchFile))


    # Imu as a seperate XML file, direct the Aircraft definition to use
    if 'Imu' in oFdm['Sensor'].keys() :
        imuFile = 'SensorImu.xml'
        ET.SubElement(elemAircraft, 'system', file = imuFile)

        SaveXml(SensorImu(oFdm), os.path.join(saveJsbPath, imuFile))


    # Gps as a seperate XML file, direct the Aircraft definition to use
    if 'Gps' in oFdm['Sensor'].keys() :
        gpsFile = 'SensorGps.xml'
        ET.SubElement(elemAircraft, 'system', file = gpsFile)

        SaveXml(SensorGps(oFdm), os.path.join(saveJsbPath, gpsFile))


    # Pitot as a seperate XML file, direct the Aircraft definition to use
    if 'Pitot' in oFdm['Sensor'].keys() :
        pitotFile = 'SensorPitot.xml'
        ET.SubElement(elemAircraft, 'system', file = pitotFile)

        SaveXml(SensorPitot(oFdm), os.path.join(saveJsbPath, pitotFile))


    # 5Hole as a seperate XML file, direct the Aircraft definition to use
    if '5Hole' in oFdm['Sensor'].keys() :
        fiveHoleFile = 'Sensor5Hole.xml'
        ET.SubElement(elemAircraft, 'system', file = fiveHoleFile)

        SaveXml(Sensor5Hole(oFdm), os.path.join(saveJsbPath, fiveHoleFile))

    # Write the Aircraft XML file
    saveFile = os.path.join(saveJsbPath, aircraftName + '.xml')
    SaveXml(elemAircraft, saveFile)


    return(elemAircraft)




#%% Table Generator, Wrapper
def TableGen(elemParent, tableArray, tableSignals, tableBreakPts):

    s = tableArray.shape
    iAxisRemList = []
    for iAxis in range(0, len(s)):
        if s[iAxis] == 1:
            iAxisRemList.append(iAxis)

    # for iRem in iAxisRemList: # XXX
    #     tableArray = tableArray.squeeze(axis=iRem)
    #     del tableSignals[iRem]
    #     del tableBreakPts[iRem]

    if len(tableArray.shape)==3:
        table = TableGen3D(elemParent, tableArray, tableSignals, tableBreakPts)
    elif len(tableArray.shape)==2:
        table = TableGen2D(elemParent, tableArray, tableSignals, tableBreakPts)
    elif (len(tableArray.shape)==1) & (tableArray.size > 1):
        table = TableGen1D(elemParent, tableArray, tableSignals, tableBreakPts)
    else:
        table = ET.SubElement(elemParent, 'value').text = str(tableArray)


    return table

#%% Table Generator, 3D
def TableGen3D(elemParent, tableArray, tableSignals, tableBreakPts):
    table = ET.SubElement(elemParent, 'table')
    #table = ET.Element('table')

    ET.SubElement(table, 'independentVar', lookup = 'row').text = tableSignals[0]
    ET.SubElement(table, 'independentVar', lookup = 'column').text = tableSignals[1]
    ET.SubElement(table, 'independentVar', lookup = 'table').text = tableSignals[2]

    indentSpace = ' '*4
    indentLvl = 4

    numRows, numColumns, numTables = np.shape(tableArray)

    columnHeader = indentSpace*(indentLvl)
    for columnVal in tableBreakPts[1]:
        columnHeader += ' '*6 + str(columnVal)


    for iTable in range(0, numTables):
        tableStr = ['\n' + columnHeader]
        for iRow in range(0, numRows):
            rowStr = str(tableArray[iRow, :, iTable]).replace('[','').replace(']','').replace('\n', '')
            tableStr.append(indentLvl*indentSpace + str(tableBreakPts[0][iRow]) + indentSpace + rowStr)

        tableStr = '\n'.join(tableStr) + '\n' + indentLvl*indentSpace # Replace list lines with '/n' strings
        ET.SubElement(table, 'tableData', breakPoint = str(tableBreakPts[2][iTable])).text = tableStr


    return table


#%% Table Generator, 2D
def TableGen2D(elemParent, tableArray, tableSignals, tableBreakPts):
    table = ET.SubElement(elemParent, 'table')

    ET.SubElement(table, 'independentVar', lookup = 'row').text = tableSignals[0]
    ET.SubElement(table, 'independentVar', lookup = 'column').text = tableSignals[1]
    indentSpace = ' '*4
    indentLvl = 4

    tableArray = tableArray.transpose()
    numRows, numColumns = np.shape(tableArray)

    columnHeader = indentSpace*(indentLvl)
    for columnVal in tableBreakPts[1]:
        columnHeader += ' '*6 + str(columnVal)

    tableStr = ['\n' + columnHeader]
    for iRow in range(0, numRows):
        rowStr = str(tableArray[iRow]).replace('[','').replace(']','').replace('\n', '')
        tableStr.append(indentLvl*indentSpace + str(tableBreakPts[0][iRow]) + indentSpace + rowStr)

    tableStr = '\n'.join(tableStr) + '\n' + indentLvl*indentSpace # Replace list lines with '/n' strings
    ET.SubElement(table, 'tableData').text = tableStr

    return table


#%% Table Generator, 1D
def TableGen1D(elemParent, tableArray, tableSignals, tableBreakPts):
    table = ET.SubElement(elemParent, 'table')

    ET.SubElement(table, 'independentVar', lookup = 'row').text = tableSignals
    indentSpace = ' '*4
    indentLvl = 4

    numRows = np.shape(tableArray)[0]

    tableStr = ['\n']
    for iRow in range(0, numRows):
        rowStr = str(tableArray[iRow]).replace('[','').replace(']','').replace('\n', '')
        tableStr.append(indentLvl*indentSpace + str(tableBreakPts[iRow]) + indentSpace + rowStr)

    tableStr = '\n'.join(tableStr) + '\n' + indentLvl*indentSpace # Replace list lines with '/n' strings
    ET.SubElement(table, 'tableData').text = tableStr

    return table


#%%
def MassBalance(oFdm):
    mass_balance = ET.Element('mass_balance')

    # Mass
    ET.SubElement(mass_balance, 'emptywt', unit = 'KG').text = str(oFdm['MassProp']['mass_kg'])

    # CG
    location = ET.SubElement(mass_balance, 'location', name = 'CG', unit = 'M')
    ET.SubElement(location, 'x').text = str(oFdm['MassProp']['rCG_S_m'][0])
    ET.SubElement(location, 'y').text = str(oFdm['MassProp']['rCG_S_m'][1])
    ET.SubElement(location, 'z').text = str(oFdm['MassProp']['rCG_S_m'][2])

    # Inertia
    ET.SubElement(mass_balance, 'ixx', unit = 'KG*M2').text = str(oFdm['MassProp']['inertia_kgm2'][0,0])
    ET.SubElement(mass_balance, 'iyy', unit = 'KG*M2').text = str(oFdm['MassProp']['inertia_kgm2'][1,1])
    ET.SubElement(mass_balance, 'izz', unit = 'KG*M2').text = str(oFdm['MassProp']['inertia_kgm2'][2,2])
    ET.SubElement(mass_balance, 'ixy', unit = 'KG*M2').text = str(oFdm['MassProp']['inertia_kgm2'][0,1])
    ET.SubElement(mass_balance, 'ixz', unit = 'KG*M2').text = str(oFdm['MassProp']['inertia_kgm2'][0,2])
    ET.SubElement(mass_balance, 'iyz', unit = 'KG*M2').text = str(oFdm['MassProp']['inertia_kgm2'][1,2])

    return(mass_balance)

#%%
def GroundReactions(oFdm):
    ground_reactions = ET.Element('ground_reactions')

    # Loop Each Gear
    for gear in oFdm['Gear'].keys():
        contact = ET.SubElement(ground_reactions, 'contact', type = 'BOGEY', name = gear)

        location = ET.SubElement(contact, 'location', unit = 'M')
        ET.SubElement(location, 'x').text = str(oFdm['Gear'][gear]['rGear_S_m'][0])
        ET.SubElement(location, 'y').text = str(oFdm['Gear'][gear]['rGear_S_m'][1])
        ET.SubElement(location, 'z').text = str(oFdm['Gear'][gear]['rGear_S_m'][2])

        ET.SubElement(contact, 'static_friction').text = str(oFdm['Gear'][gear]['FricStatic'])
        ET.SubElement(contact, 'dynamic_friction').text = str(oFdm['Gear'][gear]['FricDynamic'])
        ET.SubElement(contact, 'rolling_friction').text = str(oFdm['Gear'][gear]['FricRoll'])
        ET.SubElement(contact, 'spring_coeff', unit = 'N/M').text = str(oFdm['Gear'][gear]['kSpring_Npm'])
        ET.SubElement(contact, 'damping_coeff', unit = 'N/M/SEC').text = str(oFdm['Gear'][gear]['dampSpring_Nspm'])

        ET.SubElement(contact, 'max_steer', unit = 'DEG').text = '0.0'

    return(ground_reactions)


#%%
def Metrics(oFdm):
    metrics = ET.Element('metrics')

    # Dimensions
    ET.SubElement(metrics, 'wingarea', unit = 'M2').text = str(oFdm['Aero']['Ref']['S_m2'])
    ET.SubElement(metrics, 'wingspan', unit = 'M').text = str(oFdm['Aero']['Ref']['b_m'])
    ET.SubElement(metrics, 'chord', unit = 'M').text = str(oFdm['Aero']['Ref']['cBar_m'])

    location = ET.SubElement(metrics, 'location', name = 'AERORP', unit = 'M')
    ET.SubElement(location, 'x').text = str(oFdm['Aero']['Ref']['rAero_S_m'][0])
    ET.SubElement(location, 'y').text = str(oFdm['Aero']['Ref']['rAero_S_m'][1])
    ET.SubElement(location, 'z').text = str(oFdm['Aero']['Ref']['rAero_S_m'][2])

    location = ET.SubElement(metrics, 'location', name = 'EYEPOINT', unit = 'M')
    ET.SubElement(location, 'x').text = str(oFdm['Aero']['Ref']['rAero_S_m'][0])
    ET.SubElement(location, 'y').text = str(oFdm['Aero']['Ref']['rAero_S_m'][1])
    ET.SubElement(location, 'z').text = str(oFdm['Aero']['Ref']['rAero_S_m'][2])

    location = ET.SubElement(metrics, 'location', name = 'VRP', unit = 'M')
    ET.SubElement(location, 'x').text = '0.0'
    ET.SubElement(location, 'y').text = '0.0'
    ET.SubElement(location, 'z').text = '0.0'

    return(metrics)


#%%
def Aerodynamics(oFdm, convertFdm2Jsb):

    import copy

    # Aero Coef definitions
    coefNamesFdm = convertFdm2Jsb['Coef']['oFdm']

    # Aero Deriv dependencies definitions
    depNamesFdm = convertFdm2Jsb['Dep']['oFdm']
    depNamesJsb = convertFdm2Jsb['Dep']['jsb']
    depScale = convertFdm2Jsb['Dep']['scale']

    coefNamesFdm = convertFdm2Jsb['Coef']['oFdm']

    # Aero Breakpoint Table defintions
    indVarTable = convertFdm2Jsb['TableDef']['jsb']
    breakPtsTable = convertFdm2Jsb['TableDef']['brkPts']

    # Aero Table data to use
    aeroTable = oFdm['Aero']['Coef']

    # Define the conversion from oFdm to JSB-ML # FIXIT - switch to a CDo+CDi drag computation
    coefTable = {'CL': {'axis': 'LIFT', 'scale': None, 'type': 'force', 'deriv': 'dCL'}, \
            'CD': {'axis': 'DRAG', 'scale': None, 'type': 'force', 'deriv': 'dCD'}, \
            'CY': {'axis': 'SIDE', 'scale': None, 'type': 'force', 'deriv': 'dCY'}, \
            'CMl': {'axis': 'ROLL', 'scale': 'metrics/bw-ft', 'type': 'moment', 'deriv': 'dCMl'}, \
            'CMm': {'axis': 'PITCH', 'scale': 'metrics/cbarw-ft', 'type': 'moment', 'deriv': 'dCMm'}, \
            'CMn': {'axis': 'YAW', 'scale': 'metrics/bw-ft', 'type': 'moment', 'deriv': 'dCMn'}}


    aerodynamics = ET.Element('aerodynamics')

    #
    # Create each coefficient individually, just the table look-up
    coefNames = coefTable.keys()
    for iCoef, coef in enumerate(coefNames):
        convertCoef = coefTable[coef]

        # For each coefficient: create just the table look-up, then the Multiplication, then the summation
        for iDep, dep in enumerate(coefNamesFdm):
            function = ET.SubElement(aerodynamics, 'function', name = str('aero/coefficient/' + coef + '__' + dep))
            ET.SubElement(function, 'description').text = str(coef + '__' + dep)

            # Use the Table Generator to create the properly formated Table for JSB-ML
            tableArray = aeroTable[coef][dep]
            tableSignals = indVarTable
            tableBreakPts = breakPtsTable

            table = TableGen(function, copy.deepcopy(tableArray), copy.deepcopy(tableSignals), copy.deepcopy(tableBreakPts))

        # For each derivative: create just the table look-up, then the Multiplication, then the summation
        deriv = convertCoef['deriv']

        for iDep, dep in enumerate(depNamesFdm):
            function = ET.SubElement(aerodynamics, 'function', name = str('aero/coefficient/' + deriv + '__' + dep))
            ET.SubElement(function, 'description').text = str(deriv + '__' + dep)

            # Use the Table Generator to create the properly formated Table for JSB-ML
            tableArray = aeroTable[deriv][dep]
            tableSignals = indVarTable
            tableBreakPts = breakPtsTable

            table = TableGen(function, copy.deepcopy(tableArray), copy.deepcopy(tableSignals), copy.deepcopy(tableBreakPts))

            # Multiply each derivative by it's dependent variable
            function = ET.SubElement(aerodynamics, 'function', name = str('aero/coefficient/' + coef + '__' + dep))
            ET.SubElement(function, 'description').text = str(coef + '__' + dep + ' = ' + deriv + '__' + dep + ' * ' + dep)

            #print(coef + '__' + dep + ' = ' + deriv + '__' + dep + ' * ' + dep)

            product = ET.SubElement(function, 'product')
            ET.SubElement(product, 'property').text = 'aero/coefficient/' + deriv + '__' + dep

            #print(deriv + '__' + dep)

            depSignal = depNamesJsb[iDep]
            #print(depSignal)
            if depSignal != None:
                ET.SubElement(product, 'property').text = depSignal # Dependent Variable/Signal

            scale = depScale[iDep]
            if scale != None:
                if isinstance(scale, str):
                    ET.SubElement(product, 'property').text = str(scale) # Dependent Variable Scaling
                else:
                    ET.SubElement(product, 'value').text = str(scale) # Dependent Variable Scaling



        # Sum the Coeficients
        function = ET.SubElement(aerodynamics, 'function', name = str('aero/coefficient/' + coef))
        ET.SubElement(function, 'description').text = str(coef + ' summation')
        #print(coef + ' summation')

        summation = ET.SubElement(function, 'sum')
        for iDep, dep in enumerate(coefNamesFdm):
            ET.SubElement(summation, 'property').text = 'aero/coefficient/' + coef + '__' + dep
            #print(coef + '__' + dep)

        for iDep, dep in enumerate(depNamesFdm):
            ET.SubElement(summation, 'property').text = 'aero/coefficient/' + coef + '__' + dep
            #print(coef + '__' + dep)

    #
    # Dimensionalize the Coefficients into Forces and Moments
    for iCoef, coef in enumerate(coefNames):
        convertCoef = coefTable[coef]

        axis = ET.SubElement(aerodynamics, 'axis', name = convertCoef['axis'])

        function = ET.SubElement(axis, 'function', name = str('aero/' + convertCoef['type'] + '/' + convertCoef['axis'] + '__' + coef))
        ET.SubElement(function, 'description').text = str(convertCoef['axis'] + ' from ' + coef)

        product = ET.SubElement(function, 'product')

        ET.SubElement(product, 'property').text = 'aero/qbar-area' # qBar * sRef

        if convertCoef['scale'] != None:
            ET.SubElement(product, 'property').text = convertCoef['scale'] # Coefficient Scaling

        ET.SubElement(product, 'property').text = 'aero/coefficient/' + coef


    return(aerodynamics)

#%%
def Propulsion(oFdm):
    propulsion = ET.Element('propulsion')

    for key in oFdm['Prop'].keys():

        prop = oFdm['Prop'][key]

        # Motor/Engine
        engine = ET.SubElement(propulsion, 'engine', file = prop['nameMotor'])
#        location = ET.SubElement(engine, 'location', unit = 'M')
#        ET.SubElement(location, 'x').text = str(prop['rMotor_S_m'][0])
#        ET.SubElement(location, 'y').text = str(prop['rMotor_S_m'][1])
#        ET.SubElement(location, 'z').text = str(prop['rMotor_S_m'][2])
#        orient = ET.SubElement(engine, 'orient', unit = 'DEG')
#        ET.SubElement(orient, 'roll').text = str(prop['sMotor_deg'][0])
#        ET.SubElement(orient, 'pitch').text = str(prop['sMotor_deg'][1])
#        ET.SubElement(orient, 'yaw').text = str(prop['sMotor_deg'][2])

        # Thruster/Prop as an element of the Engine
        thruster = ET.SubElement(engine, 'thruster', file = prop['nameProp'])
        location = ET.SubElement(thruster, 'location', unit = 'M')
        ET.SubElement(location, 'x').text = str(prop['rProp_S_m'][0])
        ET.SubElement(location, 'y').text = str(prop['rProp_S_m'][1])
        ET.SubElement(location, 'z').text = str(prop['rProp_S_m'][2])
        orient = ET.SubElement(thruster, 'orient', unit = 'DEG')
        ET.SubElement(orient, 'roll').text = str(prop['sProp_deg'][0])
        ET.SubElement(orient, 'pitch').text = str(prop['sProp_deg'][1])
        ET.SubElement(orient, 'yaw').text = str(prop['sProp_deg'][2])

        ET.SubElement(thruster, 'sense').text = str(prop['sense']) # 1 = CW as viewed from cockpit, -1 = CCW
        ET.SubElement(thruster, 'p_factor').text = str(prop['p_factor'])


    return(propulsion)


#%% FCS
def FlightControl(oFdm):

    # Define all the Pilot input definition
    # Pilot Inputs, us the FG normalized sticks
    fcsPilotDef = {}
    fcsPilotDef['summer'] = {}
    fcsPilotDef['gain'] = {}

    fcsPilotDef['summer']['pilotRoll_norm'] = {}
    fcsPilotDef['summer']['pilotRoll_norm']['inputList'] = ['fcs/aileron-cmd-norm', 'fcs/roll-trim-cmd-norm']
    fcsPilotDef['summer']['pilotRoll_norm']['min'] = -1.0
    fcsPilotDef['summer']['pilotRoll_norm']['max'] = 1.0

    fcsPilotDef['gain']['cmdRoll_rps'] = {}
    fcsPilotDef['gain']['cmdRoll_rps']['input'] = 'fcs/pilotRoll_norm'
    fcsPilotDef['gain']['cmdRoll_rps']['gain'] = oFdm['FCS']['Pilot']['kRoll']

    fcsPilotDef['summer']['pilotPitch_norm'] = {}
    fcsPilotDef['summer']['pilotPitch_norm']['inputList'] = ['fcs/elevator-cmd-norm', 'fcs/pitch-trim-cmd-norm']
    fcsPilotDef['summer']['pilotPitch_norm']['min'] = -1.0
    fcsPilotDef['summer']['pilotPitch_norm']['max'] = 1.0

    fcsPilotDef['gain']['cmdPitch_rps'] = {}
    fcsPilotDef['gain']['cmdPitch_rps']['input'] = 'fcs/pilotPitch_norm'
    fcsPilotDef['gain']['cmdPitch_rps']['gain'] = oFdm['FCS']['Pilot']['kPitch']

    fcsPilotDef['summer']['pilotYaw_norm'] = {}
    fcsPilotDef['summer']['pilotYaw_norm']['inputList'] = ['fcs/rudder-cmd-norm', 'fcs/yaw-trim-cmd-norm']
    fcsPilotDef['summer']['pilotYaw_norm']['min'] = -1.0
    fcsPilotDef['summer']['pilotYaw_norm']['max'] = 1.0

    fcsPilotDef['gain']['cmdYaw_rps'] = {}
    fcsPilotDef['gain']['cmdYaw_rps']['input'] = 'fcs/pilotYaw_norm'
    fcsPilotDef['gain']['cmdYaw_rps']['gain'] = oFdm['FCS']['Pilot']['kYaw']

    fcsPilotDef['summer']['pilotFlap_norm'] = {}
    fcsPilotDef['summer']['pilotFlap_norm']['inputList'] = ['fcs/flap-cmd-norm']
    fcsPilotDef['summer']['pilotFlap_norm']['min'] = -1.0
    fcsPilotDef['summer']['pilotFlap_norm']['max'] = 1.0

    fcsPilotDef['gain']['cmdFlap_rad'] = {}
    fcsPilotDef['gain']['cmdFlap_rad']['input'] = 'fcs/pilotFlap_norm'
    fcsPilotDef['gain']['cmdFlap_rad']['gain'] = oFdm['FCS']['Pilot']['kFlap']


    # Create the JSB-ML
    elemFCS = ET.Element('flight_control', name = 'Generic Flight Control')

    pilot = ET.SubElement(elemFCS, 'channel', name = 'Pilot_Inputs')
    for type in fcsPilotDef:
        if type == 'summer':
            for key in fcsPilotDef['summer'].keys():
                entry = fcsPilotDef['summer'][key]

                summer = ET.SubElement(pilot, 'summer', name = key)

                for input in entry['inputList']:
                    ET.SubElement(summer, 'input').text = input

                if ('min' in entry.keys()) or ('max' in entry.keys()):
                    clipto = ET.SubElement(summer, 'clipto')
                    if ('min' in entry.keys()): ET.SubElement(clipto, 'min').text = str(entry['min'])
                    if ('max' in entry.keys()): ET.SubElement(clipto, 'max').text = str(entry['max'])

                ET.SubElement(summer, 'output').text = 'fcs/' + key

        if type == 'gain':
            for key in fcsPilotDef['gain'].keys():
                entry = fcsPilotDef['gain'][key]

                gain = ET.SubElement(pilot, 'pure_gain', name = key)

                ET.SubElement(gain, 'input').text = entry['input']
                ET.SubElement(gain, 'gain').text = str(entry['gain'])

                if ('min' in entry.keys()) or ('max' in entry.keys()):
                    clipto = ET.SubElement(gain, 'clipto')
                    if ('min' in entry.keys()): ET.SubElement(clipto, 'min').text = str(entry['min'])
                    if ('max' in entry.keys()): ET.SubElement(clipto, 'max').text = str(entry['max'])

                ET.SubElement(gain, 'output').text = 'fcs/' + key


    # Control System Surface Mixer
    mixer = ET.SubElement(elemFCS, 'channel', name = 'Control Mixer')

    fcsMixerDef = oFdm['FCS']['Mixer']

    for iSurf, surf in enumerate(fcsMixerDef['surfNames']):
        cmdSurf = 'cmd' + surf + '_rad'
        keyList = []
        for iInput, input in enumerate(fcsMixerDef['inputs']):
            val = fcsMixerDef['surfMix'][iSurf][iInput]

            key = input + '_2_' + surf

            if val != 0.0:
                keyList.append(key)
                gain = ET.SubElement(mixer, 'pure_gain', name = key.replace('fcs/',''))

                ET.SubElement(gain, 'input').text = 'fcs/' + input
                ET.SubElement(gain, 'gain').text = str(val)

                ET.SubElement(gain, 'output').text = 'fcs/' + key

        if any(keyList):
            summer = ET.SubElement(mixer, 'summer', name = cmdSurf)
            for key in keyList:
                ET.SubElement(summer, 'input').text = 'fcs/' + key
            ET.SubElement(summer, 'output').text = 'fcs/' + cmdSurf



    # Inputs for External Commands, this just add property to create the node in the tree
    for iSurf, surf in enumerate(fcsMixerDef['surfNames']):
        cmdSurfExt = 'cmd' + surf + '_ext_rad'
        prop = ET.SubElement(elemFCS, 'property').text = 'fcs/' + cmdSurfExt

    name = 'Motor'
    cmdMotorExt = 'cmd' + name + '_ext_nd'
    motor = ET.SubElement(elemFCS, 'property').text = 'fcs/' + cmdMotorExt # Add the Motor external command


    # Inputs for External Commands, this just add property to create the node in the tree
    extern = ET.SubElement(elemFCS, 'channel', name = 'External Input Summations')
    for iSurf, surf in enumerate(fcsMixerDef['surfNames']):
        cmdSurf = 'cmd' + surf + '_rad'
        cmdSurfExt = 'cmd' + surf + '_ext_rad'

        summer = ET.SubElement(extern, 'summer')
        ET.SubElement(summer, 'input').text = 'fcs/' + cmdSurf
        ET.SubElement(summer, 'input').text = 'fcs/' + cmdSurfExt
        ET.SubElement(summer, 'output').text = 'fcs/' + cmdSurf

    name = 'Motor'
    cmdMotor = 'cmd' + name + '_nd'
    cmdMotorExt = 'cmd' + name + '_ext_nd'
    summer = ET.SubElement(extern, 'summer')
    ET.SubElement(summer, 'input').text = 'fcs/throttle-cmd-norm'
    ET.SubElement(summer, 'input').text = 'fcs/' + cmdMotorExt
    ET.SubElement(summer, 'output').text = 'fcs/throttle-pos-norm'

    return(elemFCS)


#%% Effectors, for each surface define the 2nd order TF, and an 'actuator'
def Effectors(oFdm):

    sysEffDef = oFdm['Act']

    effectors = ET.Element('system', name = 'Effectors')
    channel = ET.SubElement(effectors, 'channel', name = 'Actuator Models')

    for surf in sysEffDef.keys():
        cmdSurf = 'cmd' + surf + '_rad'
        posSurf = 'pos' + surf + '_rad'

        entry = sysEffDef[surf]

        # Actuator - delay and freeplay
        actuator = ET.SubElement(channel, 'actuator', name = 'act' + surf)
        ET.SubElement(actuator, 'input').text = 'fcs/' + cmdSurf

        ET.SubElement(actuator, 'lag').text = str(entry['lag_nd'])
        ET.SubElement(actuator, 'hysteresis_width').text = str(entry['freeplay_rad'])
        ET.SubElement(actuator, 'delay').text = str(entry['delay_s'])

        if ('min' in entry.keys()) or ('max' in entry.keys()):
            clipto = ET.SubElement(actuator, 'clipto')
            if ('min' in entry.keys()): ET.SubElement(clipto, 'min').text = str(entry['min'])
            if ('max' in entry.keys()): ET.SubElement(clipto, 'max').text = str(entry['max'])

        ET.SubElement(actuator, 'output').text = 'fcs/' + posSurf

    return(effectors)


#%%
def Winch(oFdm):
    external_reactions = ET.Element('external_reactions')

    # Winch
    force = ET.SubElement(external_reactions, 'force', name='hitch' , frame = 'BODY', unit='N')
    location = ET.SubElement(force, 'location', unit = 'M')
    ET.SubElement(location, 'x').text = str(oFdm['Winch']['rHook_S_m'][0])
    ET.SubElement(location, 'y').text = str(oFdm['Winch']['rHook_S_m'][1])
    ET.SubElement(location, 'z').text = str(oFdm['Winch']['rHook_S_m'][2])
    direction = ET.SubElement(force, 'direction')
    ET.SubElement(direction, 'x').text = str(oFdm['Winch']['sHook_deg'][0])
    ET.SubElement(direction, 'y').text = str(oFdm['Winch']['sHook_deg'][1])
    ET.SubElement(direction, 'z').text = str(oFdm['Winch']['sHook_deg'][2])

    return(external_reactions)


#%% IMU
def SensorImu(oFdm):
    imu = ET.Element('system', name = 'Sensor - IMU')

    # Create time in us
    function = ET.SubElement(imu, 'function', name = 'sensor/imu/time_us')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'simulation/sim-time-sec'
    ET.SubElement(product, 'value').text = str(1e6)

    # Accelerometers
    if 'Accel' in oFdm['Sensor']['Imu'].keys() :
        channel = ET.SubElement(imu, 'channel', name = 'Temp Accelerometers')

        axisList = ['X', 'Y', 'Z']

        for axisName in axisList:
            accel = ET.SubElement(channel, 'accelerometer', name = 'Accel' + axisName)

            ET.SubElement(accel, 'axis').text = axisName

            location = ET.SubElement(accel, 'location', unit = 'M')
            ET.SubElement(location, 'x').text = str(oFdm['Sensor']['Imu']['Accel']['r_S_m'][0])
            ET.SubElement(location, 'y').text = str(oFdm['Sensor']['Imu']['Accel']['r_S_m'][1])
            ET.SubElement(location, 'z').text = str(oFdm['Sensor']['Imu']['Accel']['r_S_m'][2])

            orientation = ET.SubElement(accel, 'orientation', unit='DEG')
            ET.SubElement(orientation, 'roll').text = str(oFdm['Sensor']['Imu']['Accel']['s_deg'][0])
            ET.SubElement(orientation, 'pitch').text = str(oFdm['Sensor']['Imu']['Accel']['s_deg'][1])
            ET.SubElement(orientation, 'yaw').text = str(oFdm['Sensor']['Imu']['Accel']['s_deg'][2])

            ET.SubElement(accel, 'output').text = 'sensor/imu/accel' + axisName + '_true_fps2'


        # Convert Units Accelerometer to mps2
        for axisName in axisList:
            function = ET.SubElement(imu, 'function', name = 'sensor/imu/accel' + axisName + '_true_mps2')
            product = ET.SubElement(function, 'product')
            ET.SubElement(product, 'property').text = 'sensor/imu/accel' + axisName + '_true_fps2'
            ET.SubElement(product, 'value').text = str(ft2m)


        # Accelerometer Error Model
        channel = ET.SubElement(imu, 'channel', name = 'Accelerometer Error Model')

        errMod  = oFdm['Sensor']['Imu']['Accel']
        for iAxis, axisName in enumerate(axisList):
            sensor = ET.SubElement(channel, 'sensor', name = 'Accel' + axisName)
            ET.SubElement(sensor, 'input').text = 'sensor/imu/accel' + axisName + '_true_mps2'

            ET.SubElement(sensor, 'lag').text = str(errMod['lag'][iAxis])
            ET.SubElement(sensor, 'noise', variation='ABSOLUTE', distribution = 'GAUSSIAN').text = str((1.0 / 3.0) * errMod['noiseVar'][iAxis])
            ET.SubElement(sensor, 'drift_rate').text = str(errMod['drift_ps'][iAxis])
            ET.SubElement(sensor, 'gain').text = str(errMod['gain_nd'][iAxis])
            ET.SubElement(sensor, 'bias').text = str(errMod['bias'][iAxis])
            ET.SubElement(sensor, 'delay').text = str(errMod['delay_s'][iAxis])

            ET.SubElement(sensor, 'output').text = 'sensor/imu/accel' + axisName + '_mps2'


    # Gyros
    if 'Gyro' in oFdm['Sensor']['Imu'].keys() :
        errMod  = oFdm['Sensor']['Imu']['Gyro']
        channel = ET.SubElement(imu, 'channel', name = 'Gyros')

        for iAxis, axisName in enumerate(axisList):
            gyro = ET.SubElement(channel, 'gyro', name = 'Gyro' + axisName)

            ET.SubElement(gyro, 'axis').text = axisName

            location = ET.SubElement(gyro, 'location', unit = 'M')
            ET.SubElement(location, 'x').text = str(errMod['r_S_m'][0])
            ET.SubElement(location, 'y').text = str(errMod['r_S_m'][1])
            ET.SubElement(location, 'z').text = str(errMod['r_S_m'][2])

            orientation = ET.SubElement(gyro, 'orientation', unit='DEG')
            ET.SubElement(orientation, 'roll').text = str(errMod['s_deg'][0])
            ET.SubElement(orientation, 'pitch').text = str(errMod['s_deg'][1])
            ET.SubElement(orientation, 'yaw').text = str(errMod['s_deg'][2])

            ET.SubElement(gyro, 'lag').text = str(errMod['lag'][iAxis])
            ET.SubElement(gyro, 'noise', variation='ABSOLUTE', distribution = 'GAUSSIAN').text = str((1.0 / 3.0) * errMod['noiseVar'][iAxis])
            ET.SubElement(gyro, 'drift_rate').text = str(errMod['drift_ps'][iAxis])
            ET.SubElement(gyro, 'gain').text = str(errMod['gain_nd'][iAxis])
            ET.SubElement(gyro, 'bias').text = str(errMod['bias'][iAxis])
            ET.SubElement(gyro, 'delay').text = str(errMod['delay_s'][iAxis])

            ET.SubElement(gyro, 'output').text = 'sensor/imu/gyro' + axisName + '_rps'

    # Magnetometers
    if 'Mag' in oFdm['Sensor']['Imu'].keys() :
        errMod  = oFdm['Sensor']['Imu']['Mag']
        channel = ET.SubElement(imu, 'channel', name = 'Magnetometers')

        for iAxis, axisName in enumerate(axisList):
            mag = ET.SubElement(channel, 'magnetometer', name = 'Mag' + axisName)

            ET.SubElement(mag, 'axis').text = axisName

            location = ET.SubElement(mag, 'location', unit = 'M')
            ET.SubElement(location, 'x').text = str(errMod['r_S_m'][0])
            ET.SubElement(location, 'y').text = str(errMod['r_S_m'][1])
            ET.SubElement(location, 'z').text = str(errMod['r_S_m'][2])

            orientation = ET.SubElement(mag, 'orientation', unit='DEG')
            ET.SubElement(orientation, 'roll').text = str(errMod['s_deg'][0])
            ET.SubElement(orientation, 'pitch').text = str(errMod['s_deg'][1])
            ET.SubElement(orientation, 'yaw').text = str(errMod['s_deg'][2])

            ET.SubElement(mag, 'lag').text = str(errMod['lag'][iAxis])
            ET.SubElement(mag, 'noise', variation='ABSOLUTE', distribution = 'GAUSSIAN').text = str((1.0 / 3.0) * errMod['noiseVar'][iAxis])
            ET.SubElement(mag, 'drift_rate').text = str(errMod['drift_ps'][iAxis])
            ET.SubElement(mag, 'gain').text = str(errMod['gain_nd'][iAxis])
            ET.SubElement(mag, 'bias').text = str(errMod['bias'][iAxis])
            ET.SubElement(mag, 'delay').text = str(errMod['delay_s'][iAxis])

            ET.SubElement(mag, 'output').text = 'sensor/imu/mag' + axisName + '_nT'

        # Magnetometer unit conversion
        for axisName in axisList:
            function = ET.SubElement(imu, 'function', name = 'sensor/imu/mag' + axisName + '_uT')
            product = ET.SubElement(function, 'product')
            ET.SubElement(product, 'property').text = 'sensor/imu/mag' + axisName + '_nT'
            ET.SubElement(product, 'value').text = str(0.001)

    return(imu)

#%% GPS
def SensorGps(oFdm):

    gps = ET.Element('system', name = 'Sensor - GPS')

    # Create time in us
    function = ET.SubElement(gps, 'function', name = 'sensor/gps/time_us')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'simulation/sim-time-sec'
    ET.SubElement(product, 'value').text = str(1e6)

    # GPS Position
    function = ET.SubElement(gps, 'function', name = 'sensor/gps/lat_true_rad')
    ET.SubElement(function, 'property').text = 'position/lat-geod-rad'
    function = ET.SubElement(gps, 'function', name = 'sensor/gps/long_true_rad')
    ET.SubElement(function, 'property').text = 'position/long-gc-rad'

    function = ET.SubElement(gps, 'function', name = 'sensor/gps/alt_true_m')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'position/h-sl-ft'
    ET.SubElement(product, 'value').text = str(ft2m)

    # GPS Velocity
    function = ET.SubElement(gps, 'function', name = 'sensor/gps/vNorth_true_mps')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'velocities/v-north-fps'
    ET.SubElement(product, 'value').text = str(ft2m)

    function = ET.SubElement(gps, 'function', name = 'sensor/gps/vEast_true_mps')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'velocities/v-east-fps'
    ET.SubElement(product, 'value').text = str(ft2m)

    function = ET.SubElement(gps, 'function', name = 'sensor/gps/vDown_true_mps')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'velocities/v-down-fps'
    ET.SubElement(product, 'value').text = str(ft2m)


    # GPS Error Model
    channel = ET.SubElement(gps, 'channel', name = 'GPS Error Models')

    axisList = ['lat_rad', 'long_rad', 'alt_m']
    errMod  = oFdm['Sensor']['Gps']['Pos']
    for iAxis, axisName in enumerate(axisList):
        sensor = ET.SubElement(channel, 'sensor', name = axisName)

        ET.SubElement(sensor, 'input').text = 'sensor/gps/' + axisName.replace('_', '_true_')

        ET.SubElement(sensor, 'lag').text = str(errMod['lag'][iAxis])
        ET.SubElement(sensor, 'noise', variation='ABSOLUTE', distribution = 'GAUSSIAN').text = str((1.0 / 3.0) * errMod['noiseVar'][iAxis])
        ET.SubElement(sensor, 'drift_rate').text = str(errMod['drift_ps'][iAxis])
        ET.SubElement(sensor, 'gain').text = str(errMod['gain_nd'][iAxis])
        ET.SubElement(sensor, 'bias').text = str(errMod['bias'][iAxis])
        ET.SubElement(sensor, 'delay').text = str(errMod['delay_s'][iAxis])

        ET.SubElement(sensor, 'output').text = 'sensor/gps/' + axisName

    axisList = ['vNorth_mps', 'vEast_mps', 'vDown_mps']
    errMod  = oFdm['Sensor']['Gps']['Vel']
    for iAxis, axisName in enumerate(axisList):
        sensor = ET.SubElement(channel, 'sensor', name = axisName)

        ET.SubElement(sensor, 'input').text = 'sensor/gps/' + axisName.replace('_', '_true_')

        ET.SubElement(sensor, 'lag').text = str(errMod['lag'][iAxis])
        ET.SubElement(sensor, 'noise', variation='ABSOLUTE', distribution = 'GAUSSIAN').text = str((1.0 / 3.0) * errMod['noiseVar'][iAxis])
        ET.SubElement(sensor, 'drift_rate').text = str(errMod['drift_ps'][iAxis])
        ET.SubElement(sensor, 'gain').text = str(errMod['gain_nd'][iAxis])
        ET.SubElement(sensor, 'bias').text = str(errMod['bias'][iAxis])
        ET.SubElement(sensor, 'delay').text = str(errMod['delay_s'][iAxis])

        ET.SubElement(sensor, 'output').text = 'sensor/gps/' + axisName


    return(gps)

#%%

def SensorPitot(oFdm):

    pitot = ET.Element('system', name = 'Sensor - Pitot-Static Probe')

    # Create time in us
    function = ET.SubElement(pitot, 'function', name = 'sensor/pitot/time_us')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'simulation/sim-time-sec'
    ET.SubElement(product, 'value').text = str(1e6)

    # Airdata Static
    function = ET.SubElement(pitot, 'function', name = 'sensor/pitot/presStatic_true_Pa')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'atmosphere/P-psf'
    ET.SubElement(product, 'value').text = str(psf2pa)

    # Airdata Tip (Dynamic ~= Impact)
    function = ET.SubElement(pitot, 'function', name = 'sensor/pitot/presTip_true_Pa')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'aero/qbar-psf'
    ET.SubElement(product, 'value').text = str(psf2pa)

    # Airdata Temperature
    function = ET.SubElement(pitot, 'function', name = 'sensor/pitot/temp_true_C')
    product = ET.SubElement(function, 'product')
    summation = ET.SubElement(product, 'sum')
    ET.SubElement(summation, 'property').text = 'atmosphere/T-R'
    ET.SubElement(summation, 'value').text = str(-491.67)
    ET.SubElement(product, 'value').text = str(5.0/9.0)

    # Pitot Error Model
    channel = ET.SubElement(pitot, 'channel', name = 'Pitot Error Models')

    axisList = ['presStatic_Pa', 'presTip_Pa', 'temp_C']
    errMod  = oFdm['Sensor']['Gps']['Vel']
    for iAxis, axisName in enumerate(axisList):
        sensor = ET.SubElement(channel, 'sensor', name = axisName)

        ET.SubElement(sensor, 'input').text = 'sensor/pitot/' + axisName.replace('_', '_true_')

        ET.SubElement(sensor, 'lag').text = str(errMod['lag'][iAxis])
        ET.SubElement(sensor, 'noise', variation='ABSOLUTE', distribution = 'GAUSSIAN').text = str((1.0 / 3.0) * errMod['noiseVar'][iAxis])
        ET.SubElement(sensor, 'drift_rate').text = str(errMod['drift_ps'][iAxis])
        ET.SubElement(sensor, 'gain').text = str(errMod['gain_nd'][iAxis])
        ET.SubElement(sensor, 'bias').text = str(errMod['bias'][iAxis])
        ET.SubElement(sensor, 'delay').text = str(errMod['delay_s'][iAxis])

        ET.SubElement(sensor, 'output').text = 'sensor/pitot/' + axisName


    return(pitot)

#%%

def Sensor5Hole(oFdm):

    fiveHole = ET.Element('system', name = 'Sensor - 5Hole Probe')

    # Determine whether method #1 or method #2

    if 'alphaK1' and 'betaK1' in oFdm['Sensor']['5Hole'].keys():
        method = 1
    elif 'alphaK2' and 'betaK2' in oFdm['Sensor']['5Hole'].keys():
        method = 2
    else:
        print('5Hole Probe: Need either (alphaK1 and betaK1) or (alphaK2 and betaK2)')

    # Create time in us
    function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/time_us')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'simulation/sim-time-sec'
    ET.SubElement(product, 'value').text = str(1e6)

    # Airdata Static
    function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/presStatic_true_Pa')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'atmosphere/P-psf'
    ET.SubElement(product, 'value').text = str(psf2pa)

    # Airdata Tip (Dynamic ~= Impact)
    function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/presTip_true_Pa')
    product = ET.SubElement(function, 'product')
    ET.SubElement(product, 'property').text = 'aero/qbar-psf'
    ET.SubElement(product, 'value').text = str(psf2pa)

    # Airdata Temperature
    function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/temp_true_C')
    product = ET.SubElement(function, 'product')
    summation = ET.SubElement(product, 'sum')
    ET.SubElement(summation, 'property').text = 'atmosphere/T-R'
    ET.SubElement(summation, 'value').text = str(-491.67)
    ET.SubElement(product, 'value').text = str(5.0/9.0)


    # [Method 1]
    if method == 1:
        axisList = ['presStatic_Pa', 'presTip_Pa', 'presAlphaBot_Pa', 'presAlphaTop_Pa', 'presBetaRight_Pa', 'presBetaLeft_Pa', 'temp_C']

        # Alpha Difference (presAlphaBot - presAlphaTop)
        function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/presAlphaBot_true_Pa')
        product = ET.SubElement(function, 'product')
        ET.SubElement(product, 'property').text = 'aero/alpha-deg'
        ET.SubElement(product, 'property').text = 'aero/qbar-psf'
        ET.SubElement(product, 'value').text = str(psf2pa)
        ET.SubElement(product, 'value').text = str(oFdm['Sensor']['5Hole']['alphaK1'][0])

        function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/presAlphaTop_true_Pa')
        product = ET.SubElement(function, 'product')
        ET.SubElement(product, 'property').text = 'aero/alpha-deg'
        ET.SubElement(product, 'property').text = 'aero/qbar-psf'
        ET.SubElement(product, 'value').text = str(psf2pa)
        ET.SubElement(product, 'value').text = str(oFdm['Sensor']['5Hole']['alphaK1'][1])

        # [Method 2] Beta Difference (presBetaRight - presBetaLeft)
        function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/presBetaRight_true_Pa')
        product = ET.SubElement(function, 'product')
        ET.SubElement(product, 'property').text = 'aero/beta-deg'
        ET.SubElement(product, 'property').text = 'aero/qbar-psf'
        ET.SubElement(product, 'value').text = str(psf2pa)
        ET.SubElement(product, 'value').text = str(oFdm['Sensor']['5Hole']['betaK1'][0])

        function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/presBetaLeft_true_Pa')
        product = ET.SubElement(function, 'product')
        ET.SubElement(product, 'property').text = 'aero/beta-deg'
        ET.SubElement(product, 'property').text = 'aero/qbar-psf'
        ET.SubElement(product, 'value').text = str(psf2pa)
        ET.SubElement(product, 'value').text = str(oFdm['Sensor']['5Hole']['betaK1'][1])


    # [Method 2]
    elif method == 2:
        axisList = ['presStatic_Pa', 'presTip_Pa', 'presAlpha_Pa', 'presBeta_Pa', 'temp_C']

        # Alpha Difference (presAlphaBot - presAlphaTop)
        function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/presAlpha_true_Pa')
        product = ET.SubElement(function, 'product')
        ET.SubElement(product, 'property').text = 'aero/alpha-deg'
        ET.SubElement(product, 'property').text = 'aero/qbar-psf'
        ET.SubElement(product, 'value').text = str(psf2pa)
        ET.SubElement(product, 'value').text = str(oFdm['Sensor']['5Hole']['alphaK2'])

        # [Method 2] Beta Difference (presBetaRight - presBetaLeft)
        function = ET.SubElement(fiveHole, 'function', name = 'sensor/fiveHole/presBeta_true_Pa')
        product = ET.SubElement(function, 'product')
        ET.SubElement(product, 'property').text = 'aero/beta-deg'
        ET.SubElement(product, 'property').text = 'aero/qbar-psf'
        ET.SubElement(product, 'value').text = str(psf2pa)
        ET.SubElement(product, 'value').text = str(oFdm['Sensor']['5Hole']['betaK2'])


    # 5Hole Error Model
    channel = ET.SubElement(fiveHole, 'channel', name = '5Hole Error Models')

    errMod  = oFdm['Sensor']['5Hole']
    for iAxis, axisName in enumerate(axisList):
        sensor = ET.SubElement(channel, 'sensor', name = axisName)

        ET.SubElement(sensor, 'input').text = 'sensor/fiveHole/' + axisName.replace('_', '_true_')

        ET.SubElement(sensor, 'lag').text = str(errMod['lag'][iAxis])
        ET.SubElement(sensor, 'noise', variation='ABSOLUTE', distribution = 'GAUSSIAN').text = str((1.0 / 3.0) * errMod['noiseVar'][iAxis])
        ET.SubElement(sensor, 'drift_rate').text = str(errMod['drift_ps'][iAxis])
        ET.SubElement(sensor, 'gain').text = str(errMod['gain_nd'][iAxis])
        ET.SubElement(sensor, 'bias').text = str(errMod['bias'][iAxis])
        ET.SubElement(sensor, 'delay').text = str(errMod['delay_s'][iAxis])

        ET.SubElement(sensor, 'output').text = 'sensor/fiveHole/' + axisName


    return(fiveHole)
