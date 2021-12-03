"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2021 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import os.path
import numpy as np
import copy

import OpenFdm
import VspParse
import AvlWrapper


#%% Aircraft Definition
def LoadAircraftDef(load):
    oFdm = {}

    #%% Aero Data

    if 'vspPath' in load['Aero'].keys(): # Load from VSP data
        # Parse VSP files and merge the data into a single set
        vspData = VspParse.ParseAll(load['Aero']['vspPath'], load['Aero']['aircraftName'], load['Aero']['aeroName'])
        # vspData['stabTab'] is NOT a copy, values are linked!!

        # Define some of the model specific conversions for VSP to oFdm
        convertDef = {}
        convertDef['Lunit'] = 'm' # Specify the units used in the VSP model.

        # Surface name converstions
        convertDef['Surf'] = {}
        convertDef['Surf']['names'] = vspData['Stab']['surfNames']
        convertDef['Surf']['Vsp'] = ['d' + s + '_rad' for s in convertDef['Surf']['names']]
        convertDef['Surf']['Vsp_Grp'] = ['ConGrp_' + str(d) for d in range(1, 1+len(convertDef['Surf']['Vsp']))]
        convertDef['Surf']['oFdm'] = ['d' + s + '_rad' for s in convertDef['Surf']['names']]

        # Convert the VSP Aero data to oFdm
        oFdm['Aero'] = OpenFdm.LoadVsp(vspData, convertDef)
        oFdm['Aero']['surfNames'] = vspData['Stab']['surfNames']

    elif 'avlPath' in load['Aero'].keys(): # Load from AVL data

        # Load the CaseList
        loadCaseFile = os.path.join(load['Aero']['avlPath'], 'caseList.npy')
        caseList = np.load(loadCaseFile)

        ## Parse and re-package the data
        outPath = os.path.join(load['Aero']['avlPath'], 'out')
        avlOutList = AvlWrapper.ParseOut(outPath, caseList)

        avlData = AvlWrapper.Avl2Tables(avlOutList)


        # Define some of the model specific conversions
        convertDef = {}
        convertDef['Lunit'] = 'm' # Specify the units used in the AVL model.

        # Surface name converstions, AVL uses both the names and "d" surface syntax
        convertDef['Surf'] = {}
        convertDef['Surf']['Avl_D'] = ['d1','d2','d3','d4','d5','d6']
        convertDef['Surf']['names'] = ['FlapR','AilR','AilL','FlapL','Elev','Rud']
        convertDef['Surf']['Avl'] = ['dflapR','dailR','dailL','dflapL','delev','drud']
        convertDef['Surf']['oFdm'] = ['d' + s + '_rad' for s in convertDef['surf']['names']]

        oFdm['aero'] = OpenFdm.LoadAvl(avlData, convertDef)
        oFdm['aero']['surfNames'] = convertDef['surf']['names']


    #%% Mass Properties - FIXIT - Need external MassProperty Source
    oFdm['MassProp'] = {}

    # Mass Properties per OpenFlight-master repository
    oFdm['MassProp']['mass_kg'] = 1600

    cgX = -2.581
    cgY = 0.0
    cgZ = 0.216
    oFdm['MassProp']['rCG_S_m'] = np.array([cgX, cgY, cgZ])

    Ixx = oFdm['MassProp']['mass_kg']/1.959 * 0.07151
    Iyy = oFdm['MassProp']['mass_kg']/1.959 * 0.08636
    Izz = oFdm['MassProp']['mass_kg']/1.959 * 0.15364
    Ixy = 0.0
    Ixz = 0.0
    Iyz = 0.0
    oFdm['MassProp']['inertia_kgm2'] = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])


    #%% Flight Control System
    oFdm['FCS'] = {}

    # Pilot input scaling/sensitivity
    pilot = {}
    pilot['kRoll'] = 60.0 * np.pi / 180.0 # Normalized stick to cmdRoll
    pilot['kPitch'] = -45.0 * np.pi / 180.0 # Normalized stick to cmdPitch
    pilot['kYaw'] = -10.0 * np.pi / 180.0 # Normalized stick to cmdYaw
    pilot['kFlap'] = 20.0 * np.pi / 180.0 # Normalized stick to cmdFlap

    oFdm['FCS']['Pilot'] = pilot

    # Mixer
    # Get the control effectiveness matrix
    from scipy.interpolate import RegularGridInterpolator

    coefList = ['dCMl', 'dCMm', 'dCMn']
    numCoef = len(coefList)
    surfList = convertDef['Surf']['oFdm']
    numSurf = len(surfList)
    
    tblShape = oFdm['Aero']['Coef'][coefList[0]][surfList[0]].shape
    
    cntrlCoef = np.zeros(tblShape + (numCoef, numSurf))
    for iCoef, coefName in enumerate(coefList):
        for iSurf, surfName in enumerate(surfList):
            cntrlCoef[..., iCoef, iSurf] = oFdm['Aero']['Coef'][coefName][surfName]

    alphaBrkPts_deg = oFdm['Aero']['TableDef']['alphaBrkPts_deg']
    betaBrkPts_deg = oFdm['Aero']['TableDef']['betaBrkPts_deg']
    vBrkPts_mps = oFdm['Aero']['TableDef']['vBrkPts_mps']
    
    vTas_mps = 34.3 # XXX
    rho = 1.255 # XXX
    S_m2 = oFdm['Aero']['Ref']['S_m2']
    cBar_m = oFdm['Aero']['Ref']['cBar_m']
    b_m = oFdm['Aero']['Ref']['b_m']
    

    condCntrlEff = np.array([0.0, vTas_mps, 3.0])
    scaleCoef =  0.5 * rho * condCntrlEff[1]**2 * S_m2 * np.array([b_m, cBar_m, b_m])
    
    cntrlCoefInterp = RegularGridInterpolator((betaBrkPts_deg, vBrkPts_mps, alphaBrkPts_deg), cntrlCoef, method='nearest', bounds_error=False, fill_value=None)

    cntrlEff = np.atleast_2d(scaleCoef).T * cntrlCoefInterp(condCntrlEff)[0]
    
    mixer = {}
    mixer['surfNames'] = oFdm['Aero']['surfNames']
    mixer['inputs'] = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdYaw_rps']
    mixer['surfEff'] = cntrlEff

    mixer['surfMix'] = np.linalg.pinv(mixer['surfEff'])
    mixer['surfMix'][abs(mixer['surfMix'] / mixer['surfMix'].max()) < 0.05] = 0

    oFdm['FCS']['Mixer'] = mixer


    #%% Effector dynamic model, second-order with freeplay and limits
    act = {}
    for surf in oFdm['Aero']['surfNames']:
        act[surf] = {}
        
        if 'Elevator' in surf:
            act[surf]['freeplay_rad'] = 0.5 * np.pi/180.0 # Guess
            act[surf]['min'] = -25.0 * np.pi/180.0
            act[surf]['max'] = 15.0 * np.pi/180.0
            act[surf]['bandwidth_hz'] = 3.0
        elif 'Rudder' in surf:
            act[surf]['min'] = -20.0 * np.pi/180.0
            act[surf]['max'] = 20.0 * np.pi/180.0
            act[surf]['bandwidth_hz'] = 3.0

        elif 'Aileron' in surf:
            act[surf]['freeplay_rad'] = 0.5 * np.pi/180.0 # Guess
            act[surf]['min'] = -12.5 * np.pi/180.0
            act[surf]['max'] = 12.5 * np.pi/180.0
            act[surf]['bandwidth_hz'] = 3.0

        elif 'Flaps' in surf:
            act[surf]['freeplay_rad'] = 0.5 * np.pi/180.0 # Guess
            act[surf]['min'] = 0.0 * np.pi/180.0
            act[surf]['max'] = 32.0 * np.pi/180.0
            act[surf]['bandwidth_hz'] = 0.25

        act[surf]['freeplay_rad'] = 0.5 * np.pi/180.0 # Guess
        act[surf]['bandwidth_rps'] = act[surf]['bandwidth_hz'] * 2*np.pi
        act[surf]['lag_nd'] = round(200.0 / act[surf]['bandwidth_hz'])
        act[surf]['delay_s'] = 0.020
            
    oFdm['Act'] = act


    #%% Create Propulsion data (motor and prop)
    prop = {}
    prop['nameMotor'] = 'TSIO-550K'
    prop['rMotor_S_m'] = np.array([-1.292, 0.0, 0.105])
    prop['sMotor_deg'] = np.array([0, 0, 0])

    prop['nameProp'] = 'PHC-J3Y1F'
    prop['rProp_S_m'] = np.array([-0.419, 0.0, 0.105])
    prop['sProp_deg'] = np.array([0, 0, 0])

    prop['p_factor'] = 0.0
    prop['sense'] = 1.0

    oFdm['Prop'] = {}
    oFdm['Prop']['Main'] = prop


    #%% Create Sensor data
    oFdm['Sensor'] = {}

    ## IMU
    oFdm['Sensor']['Imu'] = {}

    # Accel Location and Orientation
    oFdm['Sensor']['Imu']['Accel'] = {}
    oFdm['Sensor']['Imu']['Accel']['r_S_m'] = [-1.846, 0.0, -0.297] # Center of Panel
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
    oFdm['Sensor']['Gps']['r_S_m'] = [-2.934, 0.0, -0.882] # Top of Fuselage

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
    oFdm['Sensor']['Pitot']['r_S_m'] = [-2.512, 5.675, -0.117] # Left Wingtip
    oFdm['Sensor']['Pitot']['s_deg'] = [0,0,0]

    # Airdata Error Model
    #   Pitot Vector - [presStatic_Pa, presTip_Pa, temp_C]
    oFdm['Sensor']['Pitot']['delay_s'] = [0,0,0]
    oFdm['Sensor']['Pitot']['lag'] = [0,0,0]
    oFdm['Sensor']['Pitot']['noiseVar'] = [0,0,0]
    oFdm['Sensor']['Pitot']['drift_ps'] = [0,0,0]
    oFdm['Sensor']['Pitot']['gain_nd'] = [1,1,1]
    oFdm['Sensor']['Pitot']['bias'] = [0,0,0]


    #%% Create Gear data
    mainH = 1.151
    mainY = 1.359
    mainX = -3.028

    noseH = 1.227
    noseX = -0.815

    cgX = oFdm['MassProp']['rCG_S_m'][0]

    massMain = 0.5 * oFdm['MassProp']['mass_kg'] * (noseX - cgX) / -(mainX - noseX)
    massNose = oFdm['MassProp']['mass_kg'] * (mainX - cgX) / -(noseX - cgX)
    massNose / (2*massMain)

    oFdm['Gear'] = {}
    oFdm['Gear']['Left'] = {}
    oFdm['Gear']['Left']['rGear_S_m'] = np.array([mainX, -mainY, -mainH])
    oFdm['Gear']['Left']['FricStatic'] = 0.8
    oFdm['Gear']['Left']['FricDynamic'] = 0.5
    oFdm['Gear']['Left']['FricRoll'] = 0.02

    wnDesire = 5 * 2*np.pi
    dRatio = 0.8
    oFdm['Gear']['Left']['kSpring_Npm'] = wnDesire * wnDesire * massMain
    oFdm['Gear']['Left']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massMain


    oFdm['Gear']['Right'] = copy.deepcopy(oFdm['Gear']['Left'])
    oFdm['Gear']['Right']['rGear_S_m'] = np.array([mainX, mainY, -mainH])


    oFdm['Gear']['Nose'] = copy.deepcopy(oFdm['Gear']['Left'])
    oFdm['Gear']['Nose']['rGear_S_m'] = np.array([noseX, 0.0, -noseH])

    wnDesire = 5 * 2*np.pi
    dRatio = 0.8
    oFdm['Gear']['Nose']['kSpring_Npm'] = wnDesire * wnDesire * massNose
    oFdm['Gear']['Nose']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massNose


    #%% Return
    return (oFdm)
