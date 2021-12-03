# -*- coding: utf-8 -*-
"""
Created on Wed May  6 15:04:25 2020

@author: rega0051
"""

from os import path
import time
import numpy as np
import matplotlib.pyplot as plt

import jsbsim as jsb

#%%
# Launch FGFS
#./fgfs_JSBSim.sh UltraStick25e

pathJSB = '.'
fdm = jsb.FGFDMExec(pathJSB, None)

model = 'Rocket'


#%% Setup
fdm.load_model(model)
fdm.set_dt(1/200)


fdm['simulation/integrator/rate/rotational'] = 2
fdm['simulation/integrator/rate/translational'] = 3
fdm['simulation/integrator/position/rotational'] = 3
fdm['simulation/integrator/position/rotational'] = 3


# Load IC file
fdm.load_ic('initGrnd.xml', True)

# Setup JSBSim to FlightGear
# fdm.set_output_directive(path.join('scripts', 'OutputFgfs.xml'))

# Setup JSBSim Logging
# fdm.set_output_directive(path.join('scripts', 'OutputLog.xml'))


# Display the Output
# i = 0
# while (str(fdm.get_output_filename(i),'utf-8') != ''):
#     outStr = str(fdm.get_output_filename(i),'utf-8')
#     if '/TCP' in outStr:
#         print('Output FGFS: ', outStr)
#     elif '.csv' in outStr:
#         fileLog = outStr
#         print('Output Log: ', outStr)
#     i += 1


#%% Define Controllers
import control

tFrameRate_s = fdm.get_delta_t() # Controller Frame rate

def PID2(Kp = 1, Ki = 0.0, Kd = 0, b = 1, c = 1, Tf = 0, dt = None):
    # Inputs: ['ref', 'sens']
    # Outputs: ['cmd']

    sysR = control.tf2ss(control.tf([Kp*b*Tf + Kd*c, Kp*b + Ki*Tf, Ki], [Tf, 1, 0]))
    sysY = control.tf2ss(control.tf([Kp*Tf + Kd, Kp + Ki*Tf, Ki], [Tf, 1, 0]))

    sys = control.append(sysR, sysY)

    sys.C = sys.C[0,:] - sys.C[1,:]
    sys.D = sys.D[0,:] - sys.D[1,:]

    sys.noutputs = 1
    
    if dt is not None:
        sys = control.c2d(sys, dt)

    return sys

# Roll Controller
sysCtrlRoll = PID2(0.5, Ki = 1e-6, Kd = 0.0, b = 1, c = 0, Tf = tFrameRate_s) # Ki = 0 causes issues.
sysCtrlRoll.InputName = ['refR', 'sensR']
sysCtrlRoll.OutputName = ['cmdR']



#%% Initialize
fdm.disable_output() # Disable Output
fdm.run_ic()


# Start with hold-down, this is nice for rockets
fdm['forces/hold-down'] = 1

# Set Wind
fdm['atmosphere/psiw-rad'] = 0 # Wind heading
fdm['atmosphere/wind-mag-fps'] = 0 # Wind Magnitude


#%% Run
fdm.enable_output()

t_s = [];

xCG = [];
alpha = []
weight = [];
h_ft = [];
vTrue_ftps = [];
p_rps = [];
wheel_rps = [];
drag_N = [];

tSim_s = 0
xCtrlRoll = np.matrix(np.zeros(sysCtrlRoll.nstates)).T
yCtrlRoll = np.matrix(np.zeros(1)).T
while (tSim_s <= 100):
    tSim_s = fdm.get_sim_time();
    
    if (tSim_s > 1): # Start Solid Rocket
        fdm['fcs/throttle-cmd-norm'] = 1.0

    if (fdm['inertia/weight-lbs'] < fdm['propulsion/engine/thrust-lbs']): # Release when Thrust > Weight
        fdm['forces/hold-down'] = 0

    
    # Roll Control
    refP = 0;
    sensP = fdm['velocities/p-rad_sec']
    if (tSim_s >= 10) and (fdm['propulsion/engine/thrust-lbs'] <= 1.0):
        uCtrlRoll = np.array([[refP, sensP], [refP, sensP]])
        tStep = np.array([0, tFrameRate_s])
        tCtrlRoll, yCtrlRoll, xCtrlRoll = control.forced_response(sysCtrlRoll, T = tStep, U = uCtrlRoll, X0 = xCtrlRoll[:,-1], transpose=True, return_x=True, squeeze=True)

        fdm['fcs/cmdRoll_rps'] = -yCtrlRoll[-1] # Apply the roll command


    # Air Brake Control
    if (tSim_s >= 10) and (fdm['velocities/mach'] <= 0.8): # XXX - Just to excercise the input
        fdm['fcs/cmdBrake_nd'] = 1.0 # Apply full brakes


    fdm.run()

        
    # Store some stuff for plotting
    # All the signals: fdm.query_property_catalog('')
    t_s.append(fdm['simulation/sim-time-sec'])
    weight.append(fdm['inertia/weight-lbs'])
    xCG.append(fdm['inertia/cg-x-in'])
    alpha.append(fdm['aero/alpha-deg'])
    h_ft.append(fdm['position/h-agl-ft'])
    vTrue_ftps.append(fdm['velocities/vtrue-fps'])
    p_rps.append(fdm['velocities/p-rad_sec'])
    wheel_rps.append(fdm['fcs/outRoll_rps'])
    drag_N.append(fdm['aero/coefficient/CD'])

    if (fdm['position/h-agl-ft'] <= 0): # Stop on Impact
        break

    # time.sleep(1/200) # FIXIT - ugly timer just for visuals
    
print(tSim_s)

#%% Plot
plt.figure()
plt.subplot(4,1,1)
plt.plot(t_s, h_ft); plt.grid(True)
plt.subplot(4,1,2)
plt.plot(t_s, alpha); plt.grid(True)
plt.subplot(4,1,3)
plt.plot(t_s, drag_N); plt.grid(True)
plt.subplot(4,1,4)
plt.plot(t_s, np.array(p_rps) * 60 * 2*np.pi); plt.grid(True)
plt.plot(t_s, np.array(wheel_rps) * 60 * 2*np.pi); plt.grid(True)
    
