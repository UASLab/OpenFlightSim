"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Louis Mueller, Chris Regan
"""

import os.path
import numpy as np
import copy

import OpenFdm
import VspParse
import AvlWrapper



#%% Constants
in2m = 0.0254


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
        convertDef['Lunit'] = 'in' # Specify the units used in the AVL model.

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
    mixer = {}
    mixer['surfNames'] = oFdm['Aero']['surfNames']
    mixer['inputs'] = ['cmdRoll_rps', 'cmdPitch_rps', 'cmdYaw_rps', 'cmdFlap_rad']
    mixer['surfEff'] = [
        [-1.33413, 1.33413,-0.14180,-0.56340, 0.56340, 0.00000],
        [ 0.00000, 0.00000, 0.00000, 0.00000, 0.00000,-2.27160],
        [ 0.00000, 0.00000,-1.59190, 0.00000, 0.00000, 0.00000],
        [ 0.00000, 0.00000, 0.00000, 1.00000, 1.00000, 0.00000]
        ]

    mixer['surfMix'] = np.linalg.pinv(mixer['surfEff'])
    mixer['surfMix'][abs(mixer['surfMix'] / mixer['surfMix'].max()) < 0.05] = 0

    oFdm['FCS']['Mixer'] = mixer


    #%% Effector dynamic model, second-order with freeplay and limits
    act = {}
    for surf in oFdm['Aero']['surfNames']:
        act[surf] = {}
        act[surf]['bandwidth_hz'] = 6.0
        act[surf]['bandwidth_rps'] = act[surf]['bandwidth_hz'] * 2*np.pi
        act[surf]['lag_nd'] = round(200.0 / act[surf]['bandwidth_hz']) # Based on 200 Hz Sim Frame
        act[surf]['delay_s'] = 0.020 # Guess
        act[surf]['freeplay_rad'] = 1.0 * np.pi/180.0
        act[surf]['min'] = -30.0 * np.pi/180.0
        act[surf]['max'] = 30.0 * np.pi/180.0

    oFdm['Act'] = act


    #%% Create Propulsion data (motor and prop)
    prop = {}
    prop['nameMotor'] = 'Power25'
    prop['rMotor_S_m'] = np.array([0, 0, 0])
    prop['sMotor_deg'] = np.array([0, 0, 0])

    prop['nameProp'] = 'APC 12x6e'
    prop['rProp_S_m'] = np.array([-3 * in2m, 0, 0])
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
    oFdm['Sensor']['Pitot']['r_S_m'] = [0,0,0]
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
    mainH = 7 * in2m
    mainY = 6.5 * in2m
    mainX = oFdm['MassProp']['rCG_S_m'][0] - 1.5 * in2m

    tailH = 1.5 * in2m
    tailX = 35 * in2m

    cgX = oFdm['MassProp']['rCG_S_m'][0]

    massMain = 0.5 * oFdm['MassProp']['mass_kg'] * (tailX - cgX) / -(mainX - tailX)
    massTail = oFdm['MassProp']['mass_kg'] * (mainX - cgX) / -(tailX - cgX)

    oFdm['Gear'] = {}
    oFdm['Gear']['Left'] = {}
    oFdm['Gear']['Left']['rGear_S_m'] = np.array([mainX, -mainY, -mainH])
    oFdm['Gear']['Left']['FricStatic'] = 0.8
    oFdm['Gear']['Left']['FricDynamic'] = 0.5
    oFdm['Gear']['Left']['FricRoll'] = 0.02

    wnDesire = 10 * 2*np.pi
    dRatio = 1.0
    oFdm['Gear']['Left']['kSpring_Npm'] = wnDesire * wnDesire * massMain
    oFdm['Gear']['Left']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massMain


    oFdm['Gear']['Right'] = copy.deepcopy(oFdm['Gear']['Left'])
    oFdm['Gear']['Right']['rGear_S_m'] = np.array([mainX, mainY, -mainH])


    oFdm['Gear']['Tail'] = copy.deepcopy(oFdm['Gear']['Left'])
    oFdm['Gear']['Tail']['rGear_S_m'] = np.array([tailX, 0.0, -tailH])

    wnDesire = 10 * 2*np.pi
    dRatio = 1.0
    oFdm['Gear']['Tail']['kSpring_Npm'] = wnDesire * wnDesire * massTail
    oFdm['Gear']['Tail']['dampSpring_Nspm'] = 2 * dRatio * wnDesire * massTail


    #%% Return
    return (oFdm)
