#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 12 13:18:51 2018

@author: rega0051
"""

import numpy as np


d2r = np.pi/180.0
r2d = 1.0/d2r
ft2m = 0.3049
m2ft = 1.0/ft2m

#%% Notes for Translating VSP to oFdm
def Vsp_to_Fdm(vspData, convertDef, fdmAero = {}):

    # VSP units are Kg, deg for angles, rad/sec for rates
    # oFdm units are meters, Kg, rad for angles, rad/sec for rates
    # Should only need to deal with Lunit
#%%
    Lconvert = 1.0 # Default to meters
    if 'Lunit' in convertDef.keys():
        if convertDef['Lunit'] == 'in':
            Lconvert = 0.0254

    Mconvert = 1.0
    Iconvert = 1.0
    Vconvert = 1.0
    Aconvert = np.pi / 180.0
    Sconvert = 1.0

    # Mass Properties (Avl -> oFdm)
    convertDef['massKey'] = {}
    convertDef['massKey']['Avl'] = ['mass', 'X_cg', 'Y_cg', 'Z_cg', 'Ixx', 'Iyy', 'Izz', 'Ixy', 'Iyz', 'Izx']
    convertDef['massKey']['fdm'] = ['mass_kg', 'cgX_m', 'cgY_m', 'cgZ_m', 'Ixx_kgm2', 'Iyy_kgm2', 'Izz_kgm2', 'Ixy_kgm2', 'Iyz_kgm2', 'Izx_kgm2']
    convertDef['massKey']['scale'] = [Mconvert, Lconvert, Lconvert, Lconvert, Iconvert, Iconvert, Iconvert, Iconvert, Iconvert, Iconvert]

    # Aero References (Avl -> oFdm)
    convertDef['refKey'] = {}
    convertDef['refKey']['Avl'] = ['Sref', 'Cref', 'Bref', 'Xref', 'Yref', 'Zref']
    convertDef['refKey']['fdm'] = ['S_m2', 'cBar_m', 'b_m', 'rAeroX_S_m', 'rAeroY_S_m', 'rAeroZ_S_m']
    convertDef['refKey']['scale'] = [Lconvert*Lconvert, Lconvert, Lconvert, Lconvert, Lconvert, Lconvert]

    # Conditions (VSP -> oFdm)
    convertDef['condKey'] = {}
    convertDef['condKey']['vsp'] = ['Mach_', 'AoA_', 'Beta_', 'Vinf_', 'Roll__Rate', 'Pitch_Rate', 'Yaw___Rate'] # Angles in deg, rate in rad/s, velocity in m/s
    convertDef['condKey']['fdm'] = ['mach_nd', 'alpha_rad', 'beta_rad', 'vTas_mps', 'p_rps', 'q_rps', 'r_rps']
    convertDef['condKey']['scale'] = [1.0, Aconvert, Aconvert, Vconvert, 1.0, 1.0, 1.0]

    # Coef Name Translator (VSP -> oFdm)
    convertDef['coefNames'] = {}
    convertDef['coefNames']['vsp'] = ['CL', 'CD', 'CS', 'CMl', 'CMm', 'CMn']
    convertDef['coefNames']['fdm'] = ['CL', 'CD', 'CY', 'CMl', 'CMm', 'CMn']

    # Coef Dependent Variable Translator (VSP -> oFdm)
    convertDef['coefDepNames'] = {}
    convertDef['coefDepNames']['vsp'] = ['Base_Aero', 'zero']
    convertDef['coefDepNames']['fdm'] = ['total', 'zero']

    # Derivative Name Translator (VSP -> oFdm)
    convertDef['derivNames'] = {}
    convertDef['derivNames']['vsp'] = convertDef['coefNames']['vsp']
    convertDef['derivNames']['fdm'] = ['d' + s for s in convertDef['coefNames']['fdm']]

    # Derivative Dependent Variable Translator (VSP -> oFdm)
    convertDef['derivDepNames'] = {}
    convertDef['derivDepNames']['vsp'] = ['Alpha', 'Beta', 'p', 'q', 'r'] # Angles in rad, rate in rad/s
    convertDef['derivDepNames']['fdm'] = ['alpha_rad', 'beta_rad', 'dpHat_rps', 'dqHat_rps', 'drHat_rps']
    convertDef['derivDepNames']['scale'] = [1.0, 1.0, 1.0, 1.0, 1.0]

    # Derivative Surface Names
    convertDef['derivSurfNames'] = {}
    convertDef['derivSurfNames']['vsp'] = convertDef['surf']['Vsp'] # Surface angles in rad
    convertDef['derivSurfNames']['vsp_grp'] = convertDef['surf']['Vsp_Grp']
    convertDef['derivSurfNames']['fdm'] = convertDef['surf']['fdm']
    convertDef['derivSurfNames']['scale'] = [Sconvert] * len(convertDef['derivSurfNames']['fdm'])

    #
    vSound_mps = 343

    # Aero Reference Values
    fdmAero['ref'] = {}

    fdmAero['ref']['cBar_m'] = vspData['stabTab']['cond']['Cref_'].mean()
    fdmAero['ref']['b_m'] = vspData['stabTab']['cond']['Bref_'].mean()
    fdmAero['ref']['S_m2'] = vspData['stabTab']['cond']['Sref_'].mean()

    # Aerodynamic Reference Point - VSPAero is setup to run using the CG as the RP
    refX = vspData['stabTab']['cond']['Xcg_'].mean()
    refY = vspData['stabTab']['cond']['Ycg_'].mean()
    refZ = vspData['stabTab']['cond']['Zcg_'].mean()
    fdmAero['ref']['rAero_S_m'] = np.array([refX, refY, refZ])



    fdmAero['tableDef'] = {}
    fdmAero['tableDef']['brkPtVars'] = ['beta_deg', 'vTas_mps', 'alpha_deg']
#    fdmAero['tableDef']['brkPts'] = vspData['stabTab']['tableDef']['breakIndex']
    fdmAero['tableDef']['betaBrkPts_deg'] = vspData['stabTab']['tableDef']['BetaBrkPts_']
    fdmAero['tableDef']['vBrkPts_mps'] = vspData['stabTab']['tableDef']['MachBrkPts_'] * vSound_mps
    fdmAero['tableDef']['alphaBrkPts_deg'] = vspData['stabTab']['tableDef']['AoABrkPts_']
    fdmAero['tableDef']['beta_deg'] = vspData['stabTab']['cond']['Beta_']
    fdmAero['tableDef']['vTas_mps'] = vspData['stabTab']['cond']['Mach_'] * vSound_mps
    fdmAero['tableDef']['alpha_deg'] = vspData['stabTab']['cond']['AoA_']


    # Rename the Conditions
    fdmAero['cond'] = {}
    for iCond, condFdm in enumerate(convertDef['condKey']['fdm']):
        condVsp = convertDef['condKey']['vsp'][iCond]
        fdmAero['cond'][condFdm]  = vspData['stabTab']['cond'][condVsp] * convertDef['condKey']['scale'][iCond]

    fdmAero['cond']['vTas_mps'] = fdmAero['cond']['mach_nd'] * vSound_mps
    fdmAero['cond']['dpHat_rps'] = fdmAero['cond']['p_rps'] * fdmAero['ref']['b_m'] / (2 * fdmAero['cond']['vTas_mps'])
    fdmAero['cond']['dqHat_rps'] = fdmAero['cond']['q_rps'] * fdmAero['ref']['cBar_m'] / (2 * fdmAero['cond']['vTas_mps'])
    fdmAero['cond']['drHat_rps'] = fdmAero['cond']['r_rps'] * fdmAero['ref']['b_m'] / (2 * fdmAero['cond']['vTas_mps'])


    # Add surfaces to the condition, just for kicks because they're all zero from VSP
    for surf in convertDef['derivSurfNames']['vsp']:
        fdmAero['cond'][surf]  = np.zeros_like(fdmAero['cond'][condFdm])


    # Rename the Coefficients, and Dependent Variables
    fdmAero['coef'] = {}
    for iCoef, coefFdm in enumerate(convertDef['coefNames']['fdm']):
        coefVsp = convertDef['coefNames']['vsp'][iCoef]
        fdmAero['coef'][coefFdm] = {}
        for iDep, depFdm in enumerate(convertDef['coefDepNames']['fdm']):
            depVsp = convertDef['coefDepNames']['vsp'][iDep]
            if depVsp in vspData['stabTab']['coef'][coefVsp]:
                fdmAero['coef'][coefFdm][depFdm] = vspData['stabTab']['coef'][coefVsp][depVsp]
            else:
                fdmAero['coef'][coefFdm][depFdm] = np.nan * np.ones_like(fdmAero['cond']['alpha_rad'])

    # Rename the Derivitives, and Dependent Variables
    for iDeriv, derivFdm in enumerate(convertDef['derivNames']['fdm']):
        derivVsp = convertDef['derivNames']['vsp'][iDeriv]
        fdmAero['coef'][derivFdm] = {}
        for iDep, depFdm in enumerate(convertDef['derivDepNames']['fdm']):
            depVsp = convertDef['derivDepNames']['vsp'][iDep]
            fdmAero['coef'][derivFdm][depFdm]  = vspData['stabTab']['deriv'][derivVsp][depVsp] * convertDef['derivDepNames']['scale'][iDep]

        for iSurf, surfFdm in enumerate(convertDef['derivSurfNames']['fdm']):
            surfVsp = convertDef['derivSurfNames']['vsp_grp'][iSurf]
            fdmAero['coef'][derivFdm][surfFdm]  = vspData['stabTab']['deriv'][derivVsp][surfVsp] * convertDef['derivSurfNames']['scale'][iSurf]


    # Create the Zero term
    import copy
    for iCoef, coef in enumerate(convertDef['coefNames']['fdm']):
        deriv = convertDef['derivNames']['fdm'][iCoef]

        fdmAero['coef'][coef]['zero'] = copy.deepcopy(fdmAero['coef'][coef]['total'])

        for dep in convertDef['derivDepNames']['fdm']:
            fdmAero['coef'][coef]['zero'] -= (fdmAero['coef'][deriv][dep] * fdmAero['cond'][dep])

        for surf in convertDef['derivSurfNames']['fdm']:
            fdmAero['coef'][coef]['zero'] -= (fdmAero['coef'][deriv][surf] * fdmAero['cond'][surf])

#%%
    return fdmAero


#%% Notes for Translating VSP to oFdm
def Avl_to_Fdm(avlData, convertDef, fdmAero={}):

    # AVL units are Kg, deg for angles, rad/sec for rates
    # oFdm units are meters, Kg, rad for angles, rad/sec for rates
    # Should only need to deal with Lunit
    #%%
    Lconvert = 1.0 # Default to meters
    if 'Lunit' in convertDef.keys():
        if convertDef['Lunit'] == 'in':
            Lconvert = 0.0254

    Mconvert = 1.0
    Iconvert = 1.0
    Vconvert = 1.0
    Aconvert = np.pi / 180.0
    Sconvert = np.pi / 180.0

    # Mass Properties (Avl -> oFdm)
    convertDef['massKey'] = {}
    convertDef['massKey']['Avl'] = ['mass', 'X_cg', 'Y_cg', 'Z_cg', 'Ixx', 'Iyy', 'Izz', 'Ixy', 'Iyz', 'Izx']
    convertDef['massKey']['fdm'] = ['mass_kg', 'cgX_m', 'cgY_m', 'cgZ_m', 'Ixx_kgm2', 'Iyy_kgm2', 'Izz_kgm2', 'Ixy_kgm2', 'Iyz_kgm2', 'Izx_kgm2']
    convertDef['massKey']['scale'] = [Mconvert, Lconvert, Lconvert, Lconvert, Iconvert, Iconvert, Iconvert, Iconvert, Iconvert, Iconvert]

    # Aero References (Avl -> oFdm)
    convertDef['refKey'] = {}
    convertDef['refKey']['Avl'] = ['Sref', 'Cref', 'Bref', 'Xref', 'Yref', 'Zref']
    convertDef['refKey']['fdm'] = ['S_m2', 'cBar_m', 'b_m', 'rAeroX_S_m', 'rAeroY_S_m', 'rAeroZ_S_m']
    convertDef['refKey']['scale'] = [Lconvert*Lconvert, Lconvert, Lconvert, Lconvert, Lconvert, Lconvert]

    # Conditions (Avl -> oFdm)
    convertDef['condKey'] = {}
    convertDef['condKey']['Avl'] = ['Mach', 'alpha', 'beta', 'velocity', 'pHat', 'qHat', 'rHat']
    convertDef['condKey']['fdm'] = ['mach_nd', 'alpha_rad', 'beta_rad', 'vTas_mps', 'dpHat_rps', 'dqHat_rps', 'drHat_rps']
    convertDef['condKey']['scale'] = [1.0, Aconvert, Aconvert, Vconvert, 1.0, 1.0, 1.0]

    # Coef Name Translator (Avl -> oFdm)
    convertDef['coefNames'] = {}
    convertDef['coefNames']['Avl'] = ['CL', 'CD', 'CY', 'Cl', 'Cm', 'Cn']
    convertDef['coefNames']['fdm'] = ['CL', 'CD', 'CY', 'CMl', 'CMm', 'CMn']

    # Coef Dependent Variable Translator (Avl -> oFdm)
    convertDef['coefDepNames'] = {}
    convertDef['coefDepNames']['Avl'] = ['tot', 'zero']
    convertDef['coefDepNames']['fdm'] = ['total', 'zero']

    # Derivative Name Translator (Avl -> oFdm)
    convertDef['derivNames'] = {}
    convertDef['derivNames']['Avl'] = convertDef['coefNames']['Avl']
    convertDef['derivNames']['fdm'] = ['d' + s for s in convertDef['coefNames']['fdm']]

    # Derivative Dependent Variable Translator (Avl -> oFdm)
    convertDef['derivDepNames'] = {}
    convertDef['derivDepNames']['Avl'] = ['a', 'b', 'p', 'q', 'r']
    convertDef['derivDepNames']['fdm'] = ['alpha_rad', 'beta_rad', 'dpHat_rps', 'dqHat_rps', 'drHat_rps']
    convertDef['derivDepNames']['scale'] = [1.0, 1.0, 1.0, 1.0, 1.0]

    # Derivative Surface Names
    convertDef['derivSurfNames'] = {}
    convertDef['derivSurfNames']['Avl_D'] = convertDef['surf']['Avl_D']
    convertDef['derivSurfNames']['Avl'] = convertDef['surf']['Avl'] # Surface Angles in degrees
    convertDef['derivSurfNames']['fdm'] = convertDef['surf']['fdm']
    convertDef['derivSurfNames']['scale'] = [Sconvert] * len(convertDef['derivSurfNames']['fdm'])


    # Aero Reference Values
    fdmAero['ref'] = {}
    for iRef, avlName in enumerate(convertDef['refKey']['Avl']):
        fdmName = convertDef['refKey']['fdm'][iRef]
        fdmAero['ref'][fdmName] = avlData['st'][avlName].mean() * convertDef['refKey']['scale'][iRef]

    rAeroX = fdmAero['ref']['rAeroX_S_m'].mean()
    rAeroY = fdmAero['ref']['rAeroY_S_m'].mean()
    rAeroZ = fdmAero['ref']['rAeroZ_S_m'].mean()
    fdmAero['ref']['rAero_S_m'] = np.array([rAeroX, rAeroY, rAeroZ])


    # BreakPoints
    fdmAero['tableDef'] = {}
    fdmAero['tableDef']['brkPtVars'] = ['beta_deg', 'vTas_mps', 'alpha_deg']
    fdmAero['tableDef']['brkPts'] = avlData['tableDef']['breakIndex']
    fdmAero['tableDef']['betaBrkPts_deg'] = avlData['tableDef']['betaBrkPts']
    fdmAero['tableDef']['vBrkPts_mps'] = avlData['tableDef']['velBrkPts']
    fdmAero['tableDef']['alphaBrkPts_deg'] = avlData['tableDef']['alphaBrkPts']
    fdmAero['tableDef']['beta_deg'] = avlData['tableDef']['beta']
    fdmAero['tableDef']['vTas_mps'] = avlData['tableDef']['velocity']
    fdmAero['tableDef']['alpha_deg'] = avlData['tableDef']['alpha']


    # Rename the Conditions
    fdmAero['cond'] = {}
    for iCond, avlName in enumerate(convertDef['condKey']['Avl']):
        fdmName = convertDef['condKey']['fdm'][iCond]
        fdmAero['cond'][fdmName] = avlData['run'][avlName] * convertDef['condKey']['scale'][iCond]

    # Add surfaces to the condition, just for kicks
    for iSurf, avlName in enumerate(convertDef['derivSurfNames']['Avl']):
        fdmName = convertDef['derivSurfNames']['fdm'][iSurf]
        if avlName in avlData['sb']:
            fdmAero['cond'][fdmName] = avlData['sb'][avlName] * convertDef['derivSurfNames']['scale'][iSurf]
        elif avlName in avlData['run']:
            fdmAero['cond'][fdmName] = avlData['run'][avlName] * convertDef['derivSurfNames']['scale'][iSurf]

    # Rename the Coefficients, and Dependent Variables
    fdmAero['coef'] = {}
    for iCoef, coefAvl in enumerate(convertDef['coefNames']['Avl']):
        fdmName = convertDef['coefNames']['fdm'][iCoef]
        fdmAero['coef'][fdmName] = {}
        for iDep, depAvl in enumerate(convertDef['coefDepNames']['Avl']):
            avlName = coefAvl + depAvl
            depFdm = convertDef['coefDepNames']['fdm'][iDep]
            if avlName in avlData['sb']:
                fdmAero['coef'][fdmName][depFdm] = avlData['sb'][avlName]
            elif avlName in avlData['st']:
                fdmAero['coef'][fdmName][depFdm] = avlData['st'][avlName]
            else:
                fdmAero['coef'][fdmName][depFdm] = np.nan * np.ones_like(fdmAero['cond']['alpha_rad'])


    # Rename the Derivitives, and Dependent Variables
    for iDeriv, derivAvl in enumerate(convertDef['derivNames']['Avl']):
        fdmName = convertDef['derivNames']['fdm'][iDeriv]
        fdmAero['coef'][fdmName] = {}
        for iDep, depAvl in enumerate(convertDef['derivDepNames']['Avl']):
            avlName = derivAvl + depAvl
            depFdm = convertDef['derivDepNames']['fdm'][iDep]
            if avlName in avlData['sb']:
                fdmAero['coef'][fdmName][depFdm] = avlData['sb'][avlName] * convertDef['derivDepNames']['scale'][iDep]
            elif avlName in avlData['st']:
                fdmAero['coef'][fdmName][depFdm] = avlData['st'][avlName] * convertDef['derivDepNames']['scale'][iDep]
            else:
                fdmAero['coef'][fdmName][depFdm] = np.nan * np.ones_like(fdmAero['cond']['alpha_rad'])


        for iDep, depAvl in enumerate(convertDef['derivSurfNames']['Avl_D']):
            avlName = derivAvl + depAvl
            depFdm = convertDef['derivSurfNames']['fdm'][iDep]
            if avlName in avlData['sb']:
                fdmAero['coef'][fdmName][depFdm] = avlData['sb'][avlName] * convertDef['derivSurfNames']['scale'][iDep]
            elif avlName in avlData['st']:
                fdmAero['coef'][fdmName][depFdm] = avlData['st'][avlName] * convertDef['derivSurfNames']['scale'][iDep]
            else:
                fdmAero['coef'][fdmName][depFdm] = np.nan * np.ones_like(fdmAero['cond']['alpha_rad'])


    # Create the Zero term
    import copy
    for iCoef, coef in enumerate(convertDef['coefNames']['fdm']):
        deriv = convertDef['derivNames']['fdm'][iCoef]

        fdmAero['coef'][coef]['zero'] = copy.deepcopy(fdmAero['coef'][coef]['total'])

        for dep in convertDef['derivDepNames']['fdm']:
            fdmAero['coef'][coef]['zero'] -= (fdmAero['coef'][deriv][dep] * fdmAero['cond'][dep])

        for surf in convertDef['derivSurfNames']['fdm']:
            fdmAero['coef'][coef]['zero'] -= (fdmAero['coef'][deriv][surf] * fdmAero['cond'][surf])


    return fdmAero
