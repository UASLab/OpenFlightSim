#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May  6 15:04:25 2020

@author: rega0051
"""

import jsbsim as jsb

import control
import numpy as np
import matplotlib.pyplot as plt

from os import path

#%%
def fdm_upd(t, x, u, params):
    # Parameter setup
    fdm = params.get('fdm')
    xInList = params.get('xInList')
    yList = params.get('yList')
    reset = params.get('reset', False)
    
    if reset:
        fdm.reset_to_initial_conditions(1)
    
    u0 = np.asarray([fdm[name] for name in uList])
    x0 = np.asarray([fdm[name] for name in xList])
    
    # Update the control and states
    [fdm.set_property_value(name, u[indx]) for indx, name in enumerate(uList)]
    [fdm.set_property_value(name, x[indx]) for indx, name in enumerate(xInList)]

    # Step the FDM
    fdm.enable_increment_then_hold(1)
    while (fdm.get_sim_time() <= t):
        fdm.run()
    
    # Compute the discrete updates
    dx = np.asarray([fdm[name] for name in xList])

    return dx

def fdm_out(t, x, u, params):
    # Parameter setup
    fdm = params.get('fdm')
    xInList = params.get('xInList')
    yList = params.get('yList')
    reset = params.get('reset', False)
    
    if reset:
        fdm.reset_to_initial_conditions(1)

    u0 = np.asarray([fdm[name] for name in uList])
    x0 = np.asarray([fdm[name] for name in xList])
    y0 = np.asarray([fdm[name] for name in yList])
    
    # Update the control and states
    [fdm.set_property_value(name, u[indx]) for indx, name in enumerate(uList)]
    [fdm.set_property_value(name, x[indx]) for indx, name in enumerate(xInList)]

    # Step the FDM
    fdm.enable_increment_then_hold(1)
    while (fdm.get_sim_time() <= t):
        fdm.run()
    
    # Compute the discrete updates
    y = np.asarray([fdm[name] for name in yList])
    
    return y

#%%
if 1:
    pathJSB = '/home/rega0051/Goldy3/OpenFlightSim/Simulation'
    fdm = jsb.FGFDMExec(pathJSB, None)
    
    model = 'UltraStick25e'
    
    fdm.load_model(model)
    fdm.set_dt(1/200)
    
    
    fileIC = path.join(fdm.get_aircraft_path(), model, 'initCruise.xml')
    fdm.load_ic('initCruise', True)
    fdm.run_ic()
    
    fdm['fcs/throttle-cmd-norm'] = 0.65

else:
    pathJSB = '/home/rega0051/Sim/JSBSim/jsbsim-code'
    fdm = jsb.FGFDMExec(pathJSB, None)
#    fdm.load_script('scripts/c1721.xml')
    
    model = 'c172x'
    
    fdm.load_model(model)
    
    fileIC = path.join(fdm.get_aircraft_path(), model, 'reset01.xml')
    fdm.load_ic(fileIC, False)
    fdm.run_ic()


#%%

#fdm.do_trim(1)
#fdm.get_trim_status() # Doesn't work, check fdm['aero/alpha-deg']
#fdm['ic/alpha-deg'] = fdm['aero/alpha-deg']

#uList = ['fcs/throttle-pos-norm', 'fcs/posElev_rad', 'fcs/posRud_rad', 'fcs/posAilL_rad', 'fcs/posFlapL_rad', 'fcs/posFlapR_rad', 'fcs/posAilR_rad']
#uList = ['fcs/throttle-pos-norm', 'fcs/elevator-pos-rad', 'fcs/rudder-pos-rad', 'fcs/left-aileron-pos-rad', 'fcs/right-aileron-pos-rad', 'fcs/flap-pos-rad']
#xInList = ['ic/phi-rad', 'ic/theta-rad', 'ic/psi-true-rad', 'ic/p-rad_sec', 'ic/q-rad_sec', 'ic/r-rad_sec', 'ic/u-fps', 'ic/v-fps', 'ic/w-fps', 'ic/geod-alt-ft']
#xList = ['attitude/phi-rad', 'attitude/theta-rad', 'attitude/psi-rad', 'velocities/p-rad_sec', 'velocities/q-rad_sec', 'velocities/r-rad_sec', 'velocities/u-fps', 'velocities/v-fps', 'velocities/w-fps', 'position/geod-alt-ft']
#yList = ['attitude/phi-rad', 'attitude/theta-rad', 'attitude/psi-rad', 'velocities/p-rad_sec', 'velocities/q-rad_sec', 'velocities/r-rad_sec', 'accelerations/udot-ft_sec2', 'accelerations/vdot-ft_sec2', 'accelerations/wdot-ft_sec2', 'ic/vt-fps', 'aero/beta-rad', 'aero/alpha-rad', 'position/h-sl-ft', 'flight-path/gamma-rad']

uList = ['fcs/throttle-pos-norm', 'fcs/elevator-pos-rad']
xInList = ['ic/theta-rad', 'ic/q-rad_sec', 'ic/u-fps', 'ic/w-fps', 'ic/geod-alt-ft']
xList = ['attitude/theta-rad', 'velocities/q-rad_sec', 'velocities/u-fps', 'velocities/w-fps', 'position/geod-alt-ft']
yList = ['attitude/theta-rad', 'velocities/q-rad_sec', 'accelerations/udot-ft_sec2', 'accelerations/wdot-ft_sec2', 'ic/vt-fps', 'aero/alpha-rad', 'position/h-sl-ft', 'flight-path/gamma-rad']

uInit = np.asarray([fdm[name] for name in uList])
xInInit = np.asarray([fdm[name] for name in xInList])
xInit = np.asarray([fdm[name] for name in xList])
yInit = np.asarray([fdm[name] for name in yList])

params = {}
params['fdm'] = fdm
params['reset'] = True
params['xInList'] = xInList
params['yList'] = yList

# 
dt = fdm.get_delta_t()
params['reset'] = True
fdmSysIO = control.NonlinearIOSystem(fdm_upd, fdm_out, inputs = uList, states = xList, outputs = yList, params = params, dt = 0, name='JSBSim')


# Linearize
params['reset'] = True
#xInit, uInit = control.find_eqpt(fdmSysIO, xInit, uInit, yInit, t=dt, params=params)
fdmSysLin = fdmSysIO.linearize(xInit, uInit, t=0)

#fdmSysLin.pole()
w_rps, damp, poles = fdmSysLin.damp()

#%%
# Simulate the NL system
params['reset'] = False
time_s = np.arange(0, 10, dt)
t, y = control.input_output_response(fdmSysIO, time_s, 0, xInit, params)
plt.plot(t, y[1])

# Simulate the Linear system
tLin, yLin = control.input_output_response(fdmSysLin, time_s, 0, xInit)

plt.plot(tLin, yLin[1] + yInit[1])

#%%
#fdm.set_output_directive('data_output/flightgear.xml')