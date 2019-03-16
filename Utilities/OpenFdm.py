"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Louis Mueller, Chris Regan
"""

import numpy as np


d2r = np.pi/180.0
r2d = 1.0/d2r
ft2m = 0.3049
m2ft = 1.0/ft2m

#%% Notes for Translating VSP to oFdm
def LoadVsp(vspData, convertDef, fdmAero = {}):
    # Setup Unit conversions
    # VSP units are Kg, deg for angles, rad/sec for rates
    # oFdm units are meters, Kg, rad for angles, rad/sec for rates
    # Should only need to deal with Lunit

    Lconvert = 1.0 # Default to meters
    if 'Lunit' in convertDef.keys():
        if convertDef['Lunit'] == 'in':
            Lconvert = 0.0254

    Mconvert = 1.0
    Iconvert = 1.0
    Vconvert = 1.0
    Aconvert = np.pi / 180.0 # inflow angles
    Sconvert = 1.0 # Surface angles

    # Speed of Sound, for converting bectween Mach and vTas
    vSound_mps = 343 # Default to SL
    if 'vSound_mps' in convertDef.keys():
        vSound_mps = convertDef['vSound_mps']

    # Mass Properties (Avl -> oFdm)
    convertDef['Mass'] = {}
    convertDef['Mass']['Avl'] = ['mass', 'X_cg', 'Y_cg', 'Z_cg', 'Ixx', 'Iyy', 'Izz', 'Ixy', 'Iyz', 'Izx']
    convertDef['Mass']['oFdm'] = ['mass_kg', 'cgX_m', 'cgY_m', 'cgZ_m', 'Ixx_kgm2', 'Iyy_kgm2', 'Izz_kgm2', 'Ixy_kgm2', 'Iyz_kgm2', 'Izx_kgm2']
    convertDef['Mass']['scale'] = [Mconvert, Lconvert, Lconvert, Lconvert, Iconvert, Iconvert, Iconvert, Iconvert, Iconvert, Iconvert]

    # Aero References (Avl -> oFdm)
    convertDef['Ref'] = {}
    convertDef['Ref']['Avl'] = ['Sref', 'Cref', 'Bref', 'Xref', 'Yref', 'Zref']
    convertDef['Ref']['oFdm'] = ['S_m2', 'cBar_m', 'b_m', 'rAeroX_S_m', 'rAeroY_S_m', 'rAeroZ_S_m']
    convertDef['Ref']['scale'] = [Lconvert*Lconvert, Lconvert, Lconvert, Lconvert, Lconvert, Lconvert]

    # Conditions (VSP -> oFdm)
    convertDef['Cond'] = {}
    convertDef['Cond']['vsp'] = ['Mach_', 'AoA_', 'Beta_', 'Vinf_', 'Roll__Rate', 'Pitch_Rate', 'Yaw___Rate'] # Angles in deg, rate in rad/s, velocity in m/s
    convertDef['Cond']['oFdm'] = ['mach_nd', 'alpha_rad', 'beta_rad', 'vTas_mps', 'p_rps', 'q_rps', 'r_rps']
    convertDef['Cond']['scale'] = [1.0, Aconvert, Aconvert, Vconvert, 1.0, 1.0, 1.0]

    # Coef Name Translator (VSP -> oFdm)
    convertDef['Coef'] = {}
    convertDef['Coef']['vsp'] = ['CL', 'CD', 'CS', 'CMl', 'CMm', 'CMn']
    convertDef['Coef']['oFdm'] = ['CL', 'CD', 'CY', 'CMl', 'CMm', 'CMn']

    # Coef Dependent Variable Translator (VSP -> oFdm)
    convertDef['CoefDep'] = {}
    convertDef['CoefDep']['vsp'] = ['Base_Aero', 'zero']
    convertDef['CoefDep']['oFdm'] = ['total', 'zero']

    # Derivative Name Translator (VSP -> oFdm)
    convertDef['Deriv'] = {}
    convertDef['Deriv']['vsp'] = convertDef['Coef']['vsp']
    convertDef['Deriv']['oFdm'] = ['d' + s for s in convertDef['Coef']['oFdm']]

    # Derivative Dependent Variable Translator (VSP -> oFdm)
    convertDef['DerivDep'] = {}
    convertDef['DerivDep']['vsp'] = ['Alpha', 'Beta', 'p', 'q', 'r'] # Angles in rad, rate in rad/s
    convertDef['DerivDep']['oFdm'] = ['alpha_rad', 'beta_rad', 'dpHat_rps', 'dqHat_rps', 'drHat_rps']
    convertDef['DerivDep']['scale'] = [1.0, 1.0, 1.0, 1.0, 1.0]

    # Derivative Surface Names
    convertDef['DerivSurf'] = {}
    convertDef['DerivSurf']['vsp'] = convertDef['Surf']['Vsp'] # Surface angles in rad
    convertDef['DerivSurf']['vsp_grp'] = convertDef['Surf']['Vsp_Grp']
    convertDef['DerivSurf']['oFdm'] = convertDef['Surf']['oFdm']
    convertDef['DerivSurf']['scale'] = [Sconvert] * len(convertDef['DerivSurf']['oFdm'])


    # Aero Reference Values
    fdmAero['Ref'] = {}

    fdmAero['Ref']['cBar_m'] = vspData['StabTab']['Cond']['Cref_'].mean()
    fdmAero['Ref']['b_m'] = vspData['StabTab']['Cond']['Bref_'].mean()
    fdmAero['Ref']['S_m2'] = vspData['StabTab']['Cond']['Sref_'].mean()

    # Aerodynamic Reference Point - VSPAero is setup to run using the CG as the RP
    refX = vspData['StabTab']['Cond']['Xcg_'].mean()
    refY = vspData['StabTab']['Cond']['Ycg_'].mean()
    refZ = vspData['StabTab']['Cond']['Zcg_'].mean()
    fdmAero['Ref']['rAero_S_m'] = np.array([refX, refY, refZ])


    # Create Table Definitions
    fdmAero['TableDef'] = {}
    fdmAero['TableDef']['brkPtVars'] = ['beta_deg', 'vTas_mps', 'alpha_deg']
#    fdmAero['TableDef']['brkPts'] = vspData['StabTab']['TableDef']['breakIndex']
    fdmAero['TableDef']['betaBrkPts_deg'] = vspData['StabTab']['TableDef']['BetaBrkPts_']
    fdmAero['TableDef']['vBrkPts_mps'] = vspData['StabTab']['TableDef']['MachBrkPts_'] * vSound_mps
    fdmAero['TableDef']['alphaBrkPts_deg'] = vspData['StabTab']['TableDef']['AoABrkPts_']
    fdmAero['TableDef']['beta_deg'] = vspData['StabTab']['Cond']['Beta_']
    fdmAero['TableDef']['vTas_mps'] = vspData['StabTab']['Cond']['Mach_'] * vSound_mps
    fdmAero['TableDef']['alpha_deg'] = vspData['StabTab']['Cond']['AoA_']


    # Rename the Conditions
    fdmAero['Cond'] = {}
    for iCond, condFdm in enumerate(convertDef['Cond']['oFdm']):
        condVsp = convertDef['Cond']['vsp'][iCond]
        fdmAero['Cond'][condFdm]  = vspData['StabTab']['Cond'][condVsp] * convertDef['Cond']['scale'][iCond]

    fdmAero['Cond']['vTas_mps'] = fdmAero['Cond']['mach_nd'] * vSound_mps
    fdmAero['Cond']['dpHat_rps'] = fdmAero['Cond']['p_rps'] * fdmAero['Ref']['b_m'] / (2 * fdmAero['Cond']['vTas_mps'])
    fdmAero['Cond']['dqHat_rps'] = fdmAero['Cond']['q_rps'] * fdmAero['Ref']['cBar_m'] / (2 * fdmAero['Cond']['vTas_mps'])
    fdmAero['Cond']['drHat_rps'] = fdmAero['Cond']['r_rps'] * fdmAero['Ref']['b_m'] / (2 * fdmAero['Cond']['vTas_mps'])


    # Add surfaces to the condition, just for kicks because they're all zero from VSP
    for surf in convertDef['DerivSurf']['vsp']:
        fdmAero['Cond'][surf]  = np.zeros_like(fdmAero['Cond'][condFdm])


    # Rename the Coefficients, and Dependent Variables
    fdmAero['Coef'] = {}
    for iCoef, coefFdm in enumerate(convertDef['Coef']['oFdm']):
        coefVsp = convertDef['Coef']['vsp'][iCoef]
        fdmAero['Coef'][coefFdm] = {}
        for iDep, depFdm in enumerate(convertDef['CoefDep']['oFdm']):
            depVsp = convertDef['CoefDep']['vsp'][iDep]
            if depVsp in vspData['StabTab']['Coef'][coefVsp]:
                fdmAero['Coef'][coefFdm][depFdm] = vspData['StabTab']['Coef'][coefVsp][depVsp]
            else:
                fdmAero['Coef'][coefFdm][depFdm] = np.nan * np.ones_like(fdmAero['Cond']['alpha_rad'])

    # Rename the Derivitives, and Dependent Variables
    for iDeriv, derivFdm in enumerate(convertDef['Deriv']['oFdm']):
        derivVsp = convertDef['Deriv']['vsp'][iDeriv]
        fdmAero['Coef'][derivFdm] = {}
        for iDep, depFdm in enumerate(convertDef['DerivDep']['oFdm']):
            depVsp = convertDef['DerivDep']['vsp'][iDep]
            fdmAero['Coef'][derivFdm][depFdm]  = vspData['StabTab']['Deriv'][derivVsp][depVsp] * convertDef['DerivDep']['scale'][iDep]

        for iSurf, surfFdm in enumerate(convertDef['DerivSurf']['oFdm']):
            surfVsp = convertDef['DerivSurf']['vsp_grp'][iSurf]
            fdmAero['Coef'][derivFdm][surfFdm]  = vspData['StabTab']['Deriv'][derivVsp][surfVsp] * convertDef['DerivSurf']['scale'][iSurf]


    # Create the Zero term
    import copy
    for iCoef, coef in enumerate(convertDef['Coef']['oFdm']):
        deriv = convertDef['Deriv']['oFdm'][iCoef]

        fdmAero['Coef'][coef]['zero'] = copy.deepcopy(fdmAero['Coef'][coef]['total'])

        for dep in convertDef['DerivDep']['oFdm']:
            fdmAero['Coef'][coef]['zero'] -= (fdmAero['Coef'][deriv][dep] * fdmAero['Cond'][dep])

        for surf in convertDef['DerivSurf']['oFdm']:
            fdmAero['Coef'][coef]['zero'] -= (fdmAero['Coef'][deriv][surf] * fdmAero['Cond'][surf])

    # Return the fdmAero Dictionary
    return fdmAero


#%% Convert AVL data to oFdm
def LoadAvl(avlData, convertDef, fdmAero={}):
    # AVL units are Kg, deg for angles, rad/sec for rates
    # oFdm units are meters, Kg, rad for angles, rad/sec for rates
    # Should only need to deal with Lunit

    Lconvert = 1.0 # Default to meters
    if 'Lunit' in convertDef.keys():
        if convertDef['Lunit'] == 'in':
            Lconvert = 0.0254

    Mconvert = 1.0
    Iconvert = 1.0
    Vconvert = 1.0
    Aconvert = np.pi / 180.0 # inflow angles
    Sconvert = np.pi / 180.0 # surface angles

    # Mass Properties (Avl -> oFdm)
    convertDef['Mass'] = {}
    convertDef['Mass']['Avl'] = ['mass', 'X_cg', 'Y_cg', 'Z_cg', 'Ixx', 'Iyy', 'Izz', 'Ixy', 'Iyz', 'Izx']
    convertDef['Mass']['oFdm'] = ['mass_kg', 'cgX_m', 'cgY_m', 'cgZ_m', 'Ixx_kgm2', 'Iyy_kgm2', 'Izz_kgm2', 'Ixy_kgm2', 'Iyz_kgm2', 'Izx_kgm2']
    convertDef['Mass']['scale'] = [Mconvert, Lconvert, Lconvert, Lconvert, Iconvert, Iconvert, Iconvert, Iconvert, Iconvert, Iconvert]

    # Aero References (Avl -> oFdm)
    convertDef['Ref'] = {}
    convertDef['Ref']['Avl'] = ['Sref', 'Cref', 'Bref', 'Xref', 'Yref', 'Zref']
    convertDef['Ref']['oFdm'] = ['S_m2', 'cBar_m', 'b_m', 'rAeroX_S_m', 'rAeroY_S_m', 'rAeroZ_S_m']
    convertDef['Ref']['scale'] = [Lconvert*Lconvert, Lconvert, Lconvert, Lconvert, Lconvert, Lconvert]

    # Conditions (Avl -> oFdm)
    convertDef['Cond'] = {}
    convertDef['Cond']['Avl'] = ['Mach', 'alpha', 'beta', 'velocity', 'pHat', 'qHat', 'rHat']
    convertDef['Cond']['oFdm'] = ['mach_nd', 'alpha_rad', 'beta_rad', 'vTas_mps', 'dpHat_rps', 'dqHat_rps', 'drHat_rps']
    convertDef['Cond']['scale'] = [1.0, Aconvert, Aconvert, Vconvert, 1.0, 1.0, 1.0]

    # Coef Name Translator (Avl -> oFdm)
    convertDef['Coef'] = {}
    convertDef['Coef']['Avl'] = ['CL', 'CD', 'CY', 'Cl', 'Cm', 'Cn']
    convertDef['Coef']['oFdm'] = ['CL', 'CD', 'CY', 'CMl', 'CMm', 'CMn']

    # Coef Dependent Variable Translator (Avl -> oFdm)
    convertDef['CoefDep'] = {}
    convertDef['CoefDep']['Avl'] = ['tot', 'zero']
    convertDef['CoefDep']['oFdm'] = ['total', 'zero']

    # Derivative Name Translator (Avl -> oFdm)
    convertDef['Deriv'] = {}
    convertDef['Deriv']['Avl'] = convertDef['Coef']['Avl']
    convertDef['Deriv']['oFdm'] = ['d' + s for s in convertDef['Coef']['oFdm']]

    # Derivative Dependent Variable Translator (Avl -> oFdm)
    convertDef['DerivDep'] = {}
    convertDef['DerivDep']['Avl'] = ['a', 'b', 'p', 'q', 'r']
    convertDef['DerivDep']['oFdm'] = ['alpha_rad', 'beta_rad', 'dpHat_rps', 'dqHat_rps', 'drHat_rps']
    convertDef['DerivDep']['scale'] = [1.0, 1.0, 1.0, 1.0, 1.0]

    # Derivative Surface Names
    convertDef['DerivSurf'] = {}
    convertDef['DerivSurf']['Avl_D'] = convertDef['surf']['Avl_D']
    convertDef['DerivSurf']['Avl'] = convertDef['surf']['Avl'] # Surface Angles in degrees
    convertDef['DerivSurf']['oFdm'] = convertDef['surf']['oFdm']
    convertDef['DerivSurf']['scale'] = [Sconvert] * len(convertDef['DerivSurf']['oFdm'])


    # Aero Reference Values
    fdmAero['Ref'] = {}
    for iRef, avlName in enumerate(convertDef['Ref']['Avl']):
        fdmName = convertDef['Ref']['oFdm'][iRef]
        fdmAero['Ref'][fdmName] = avlData['st'][avlName].mean() * convertDef['Ref']['scale'][iRef]

    rAeroX = fdmAero['Ref']['rAeroX_S_m'].mean()
    rAeroY = fdmAero['Ref']['rAeroY_S_m'].mean()
    rAeroZ = fdmAero['Ref']['rAeroZ_S_m'].mean()
    fdmAero['Ref']['rAero_S_m'] = np.array([rAeroX, rAeroY, rAeroZ])


    # BreakPoints
    fdmAero['TableDef'] = {}
    fdmAero['TableDef']['brkPtVars'] = ['beta_deg', 'vTas_mps', 'alpha_deg']
    fdmAero['TableDef']['brkPts'] = avlData['TableDef']['breakIndex']
    fdmAero['TableDef']['betaBrkPts_deg'] = avlData['TableDef']['betaBrkPts']
    fdmAero['TableDef']['vBrkPts_mps'] = avlData['TableDef']['velBrkPts']
    fdmAero['TableDef']['alphaBrkPts_deg'] = avlData['TableDef']['alphaBrkPts']
    fdmAero['TableDef']['beta_deg'] = avlData['TableDef']['beta']
    fdmAero['TableDef']['vTas_mps'] = avlData['TableDef']['velocity']
    fdmAero['TableDef']['alpha_deg'] = avlData['TableDef']['alpha']


    # Rename the Conditions
    fdmAero['Cond'] = {}
    for iCond, avlName in enumerate(convertDef['Cond']['Avl']):
        fdmName = convertDef['Cond']['oFdm'][iCond]
        fdmAero['Cond'][fdmName] = avlData['run'][avlName] * convertDef['Cond']['scale'][iCond]

    # Add surfaces to the condition, just for kicks
    for iSurf, avlName in enumerate(convertDef['DerivSurf']['Avl']):
        fdmName = convertDef['DerivSurf']['oFdm'][iSurf]
        if avlName in avlData['sb']:
            fdmAero['Cond'][fdmName] = avlData['sb'][avlName] * convertDef['DerivSurf']['scale'][iSurf]
        elif avlName in avlData['run']:
            fdmAero['Cond'][fdmName] = avlData['run'][avlName] * convertDef['DerivSurf']['scale'][iSurf]

    # Rename the Coefficients, and Dependent Variables
    fdmAero['Coef'] = {}
    for iCoef, coefAvl in enumerate(convertDef['Coef']['Avl']):
        fdmName = convertDef['Coef']['oFdm'][iCoef]
        fdmAero['Coef'][fdmName] = {}
        for iDep, depAvl in enumerate(convertDef['CoefDep']['Avl']):
            avlName = coefAvl + depAvl
            depFdm = convertDef['CoefDep']['oFdm'][iDep]
            if avlName in avlData['sb']:
                fdmAero['Coef'][fdmName][depFdm] = avlData['sb'][avlName]
            elif avlName in avlData['st']:
                fdmAero['Coef'][fdmName][depFdm] = avlData['st'][avlName]
            else:
                fdmAero['Coef'][fdmName][depFdm] = np.nan * np.ones_like(fdmAero['Cond']['alpha_rad'])


    # Rename the Derivitives, and Dependent Variables
    for iDeriv, derivAvl in enumerate(convertDef['Deriv']['Avl']):
        fdmName = convertDef['Deriv']['oFdm'][iDeriv]
        fdmAero['Coef'][fdmName] = {}
        for iDep, depAvl in enumerate(convertDef['DerivDep']['Avl']):
            avlName = derivAvl + depAvl
            depFdm = convertDef['DerivDep']['oFdm'][iDep]
            if avlName in avlData['sb']:
                fdmAero['Coef'][fdmName][depFdm] = avlData['sb'][avlName] * convertDef['DerivDep']['scale'][iDep]
            elif avlName in avlData['st']:
                fdmAero['Coef'][fdmName][depFdm] = avlData['st'][avlName] * convertDef['DerivDep']['scale'][iDep]
            else:
                fdmAero['Coef'][fdmName][depFdm] = np.nan * np.ones_like(fdmAero['Cond']['alpha_rad'])


        for iDep, depAvl in enumerate(convertDef['DerivSurf']['Avl_D']):
            avlName = derivAvl + depAvl
            depFdm = convertDef['DerivSurf']['oFdm'][iDep]
            if avlName in avlData['sb']:
                fdmAero['Coef'][fdmName][depFdm] = avlData['sb'][avlName] * convertDef['DerivSurf']['scale'][iDep]
            elif avlName in avlData['st']:
                fdmAero['Coef'][fdmName][depFdm] = avlData['st'][avlName] * convertDef['DerivSurf']['scale'][iDep]
            else:
                fdmAero['Coef'][fdmName][depFdm] = np.nan * np.ones_like(fdmAero['Cond']['alpha_rad'])


    # Create the Zero term
    import copy
    for iCoef, coef in enumerate(convertDef['Coef']['oFdm']):
        deriv = convertDef['Deriv']['oFdm'][iCoef]

        fdmAero['Coef'][coef]['zero'] = copy.deepcopy(fdmAero['Coef'][coef]['total'])

        for dep in convertDef['DerivDep']['oFdm']:
            fdmAero['Coef'][coef]['zero'] -= (fdmAero['Coef'][deriv][dep] * fdmAero['Cond'][dep])

        for surf in convertDef['DerivSurf']['oFdm']:
            fdmAero['Coef'][coef]['zero'] -= (fdmAero['Coef'][deriv][surf] * fdmAero['Cond'][surf])


    return fdmAero
