# -*- coding: utf-8 -*-
"""
Created on Wed May  6 15:04:25 2020

@author: rega0051
"""

from os import path
import time
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


#%% Initialize
fdm.disable_output() # Disable Output
fdm.run_ic()

# Start with hold-down, this is nice for rockets
fdm['forces/hold-down'] = 1


#%% Run
fdm.enable_output()

t_s = [];
xCG = [];
alpha = []
weight = [];
h_ft = [];
vTrue_ftps = [];
p_rps = [];

while (fdm.get_sim_time() <= 100):
    if (fdm.get_sim_time() > 1): # Start Solid Rocket
        fdm['fcs/throttle-cmd-norm'] = 1.0

    if (fdm['inertia/weight-lbs'] < fdm['propulsion/engine/thrust-lbs']): # Release
        fdm['forces/hold-down'] = 0

    fdm.run()
    
    # All the signals: fdm.query_property_catalog('')
    t_s.append(fdm['simulation/sim-time-sec'])
    weight.append(fdm['inertia/weight-lbs'])
    xCG.append(fdm['inertia/cg-x-in'])
    alpha.append(fdm['aero/alpha-deg'])
    h_ft.append(fdm['position/h-agl-ft'])
    vTrue_ftps.append(fdm['velocities/vtrue-fps'])
    p_rps.append(fdm['velocities/p-rad_sec'])

    if (fdm['position/h-agl-ft'] <= 0): # Impact
        break

    # time.sleep(1/200) # FIXIT - ugly timer just for visuals
    
print(fdm.get_sim_time())

#%% Plot
plt.figure()
plt.subplot(3,1,1)
plt.plot(t_s, h_ft); plt.grid(True)
plt.subplot(3,1,2)
plt.plot(t_s, vTrue_ftps); plt.grid(True)
plt.subplot(3,1,3)
plt.plot(t_s, p_rps); plt.grid(True)
    
