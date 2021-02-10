# -*- coding: utf-8 -*-
"""
Created on Wed May  6 15:04:25 2020

@author: rega0051
"""

from os import path
import time

import jsbsim as jsb

#%%
# Launch FGFS
#./fgfs_JSBSim.sh UltraStick25e

pathJSB = '.'
fdm = jsb.FGFDMExec(pathJSB, None)

model = 'UltraStick25e'


#%% Setup
fdm.load_model(model)
fdm.set_dt(1/200)

# Load IC file
#fileIC = path.join(fdm.get_aircraft_path(), model, 'initCruise.xml')
fdm.load_ic('initCruise.xml', True)

# Setup JSBSim to FlightGear
fdm.set_output_directive(path.join('scripts', 'OutputFgfs.xml'))

# Setup JSBSim Logging
fdm.set_output_directive(path.join('scripts', 'OutputLog.xml'))


# Display the Output
i = 0
while (str(fdm.get_output_filename(i),'utf-8') != ''):
    outStr = str(fdm.get_output_filename(i),'utf-8')
    if '/TCP' in outStr:
        print('Output FGFS: ', outStr)
    elif '.csv' in outStr:
        fileLog = outStr
        print('Output Log: ', outStr)
    i += 1


#%% Initialize
fdm.disable_output() # Disable Output
fdm.run_ic()

fdm['fcs/throttle-cmd-norm'] = 0.65
fdm.run()
# fdm.do_trim(1)
fdm.get_trim_status()

print('Alpha: ', fdm['aero/alpha-deg'])

#%% Run
fdm.enable_output()

# Step the FDM
#fdm.enable_increment_then_hold(1)
while (fdm.get_sim_time() <= 10):
    fdm.run()

    time.sleep(1/200) # FIXIT - ugly timer just for visuals
    
print(fdm.get_sim_time())
    
#%% Load JSBSim Log Data
import pandas as pd
dataLog = pd.read_csv(fileLog, index_col=0)
