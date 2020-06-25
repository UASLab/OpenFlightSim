# OpenFlightSim
Open Flight Simulation (oFS)

www.uav.aem.umn.edu


oFS is a companion to OpenFlightAnalysis for data analysis and RAPTRS for flight software.

https://github.com/UASLab/OpenFlightAnalysis

https://github.com/bolderflight/RAPTRS


oFS is intended as a tool to generate and simulate flight vehicles. Currently, the tools use OpenVSP and/or AVL as sources for Aerodynamic data. The aerodynamic data is parsed and augmented with sub-system specification (propulsion, actuation, etc.) into an intermediate format known as oFDM (Open Flight Dynamics Model). The oFDM can be used to generate a complete simulation definition for JSBSim to make a flyable aircraft model. JSBSim models are augmented with addition I/O capability to allow for algorithm-in-the-loop, software-in-the-loop (SIL), and hardware-in-the-loop (HIL) testing. SIL and HIL testing relies on RAPTRS for flight control.

http://web.mit.edu/drela/Public/web/avl/

http://openvsp.org/

https://github.com/JSBSim-Team/jsbsim

http://home.flightgear.org/download/

# Modes of Operation
Common:
JSBSim executes with Python bindings. (Works on Linux and WSL, does not work in native Windows)
Pipe to Flightgear for visuals, if desired, in all modes. (Works accross TCP, seemless on Linux and Windows)

## Algorithm in the Loop (AIL)
(Not tested on Linux, not tested on Windows)
Use Python to provide control. No messaging, no FMU-like behavior. Just read the desired JSBSim properties, execute your algorithm, write to JSBSim properties.

## Software in the Loop (SIL) 
(Works on Linux and Windows)
RAPTRS interfaces with JSBSim through fmu_message definitions, with FMU like behavior emulated in Python. RAPTRS built for executing in native Linux (works with Windows Linux Subsystem as well). RAPTRS does require a valid config .json file to run.

## Processor in the Loop (PIL) 
(Works on Linux, not tested on Windows)
Similar to SIL. But RAPTRS runs on an SOC (BeagleBoneBlack) and interfaces with the Host machine via USB. RAPTRS is built for the SOC, check realtime execution of RAPTRS. Data logging tests and Telemtry can be sent.

## Hardware in the Loop (HIL) or Aircraft in the Loop (AIL) 
(Not tested on Linux, not tested on Windows)
Similar to PIL. SOC is connected to a hardware FMU. individual FMU<->SOC messages are controlled such that the FMU/SOC interface can be checked while still allowing simulated sensor data to be received by the SOC.

Final execution testing is still required on the Aircraft; HIL/PIL testing will alter some of the execution timing slightly.

# Install
## Linux (Debian 10.4):
### JSBSim
```
mkdir Sim; cd Sim; mkdir JSBSim; cd JSBSim
sudo apt-get install git
git clone https://github.com/JSBSim-Team/jsbsim.git jsbsim-repo
mkdir build; cd build;
sudo apt-get install cmake make g++
sudo apt-get install python3 cython3 python3-setuptools
cmake ../jsbsim-repo/ -DCYTHON_EXECUTABLE=/usr/bin/cython3 -DINSTALL_PYTHON_MODULE=ON
make JSBSim
make PythonJSBSim
make install
make test
```

### RAPTRS (minimal for compiling SOC code for AMD64)
```
cd Goldy3;
git clone https://github.com/UASLab/RAPTRS.git
cd RAPTRS
git checkout SimOverhaul

sudo apt-get install g++ libEigen3-dev

cd software;
make flight_amd64
make datalog_amd64
```

### Flightgear
```
sudo apt-get install flightgear
```

### OpenFlightSim
```
mkdir Goldy3; cd Goldy3
git clone https://github.com/UASLab/OpenFlightSim.git
cd OpenFlightSim
git checkout SimOverhaul
cd Simulation
sudo apt-get install python3-numpy python3-pandas
```

Tests:
(start FGFS on windows, then in WSL2...)
```JSBSim scripts/jsb_UltraStick25e_Cruise.xml```
(should run for 100 seconds)

(start FGFS on windows, then in WSL2...)
```python3 python/JSBSim_Script_Demo.py```
(run without error for 10 seconds, displays a 10)

### Configs
```
cd ~/Goldy3;
git clone https://github.umn.edu/UAV-Lab/Config.git
```

### Tests
```
sudo apt-get install socat netcat
sudo apt-get install python3-pygame python3-serial
```

SIL TEST
(multiple terminals, all at: ~/Goldy/OpenFligtSim/Simulation)
```./fgfs_JSBSim.sh UltraStick25e```
```./start_SIL_Comm.sh```
```python3 python/JSBSim_SIL_Demo.py ```
```~/Goldy3/RAPTRS/software/bin/flight_amd64 ~/Goldy3/Config/thor.json```

##  Windows 10 with Windows Linux Subsystem - (Debian 10.4)
First, install WLS2 and Debian (https://docs.microsoft.com/en-us/windows/wsl/install-win10)
In the WLS2-Debian:
ln -s /mnt/c/Users/rega0051/Documents/Goldy3/ Goldy3

### Flightgear
Install FlightGear in Windows10 (not WSL2). (https://www.flightgear.org/download/) Tested with version 2018.3.5.
Need to add FlightGear###/bin to the Windows Environment Path... 

### JSBSim in Windows with MSCV (Use JSBSim release https://github.com/JSBSim-Team/jsbsim/releases)
On Windows I use Anaconda as the Python Package manager, however pygame and JSBSim aren't in conda
pip install pygame

https://github.com/JSBSim-Team/jsbsim/releases
pip install jsbsim --no-index -f "https://github.com/JSBSim-Team/jsbsim/releases/download/Rolling-release-v2019/JSBSim-1.1.0.dev1-735-cp37-cp37m-win_amd64.whl" 

(No clue how to actually build with MSCV)

then, with a "conda" command window (These work)
```python python\Joystick_Demo.py```
```python python\JSBSim_Script_Demo.py```

### RAPTRS in WSL2-Debian (minimal for compiling SOC code for AMD64)
use GitHub Desktop to clone to: Goldy3/RAPTRS
(then same and Linux)

### Configs
use GitHub Desktop to clone to: Goldy3/Config
(then same as Linux)

## OpenFlightSim
use GitHub Desktop to clone to: Goldy3/OpenFlightSim


### Tests

SIL TEST
(start FGFS on Windows)

In WSL2:
```./start_SIL_Comm.sh```

conda command window:
```python python\JSBSim_SIL_Demo.py```

In another WSL2:
```~/Goldy3/RAPTRS/software/bin/flight_amd64 ~/Goldy3/Config/thor.json```


# Not Working:

### JSBSim with WLS2-Debian
(same as Linux install)
(Joystick won't work with WSL2!!)

### JSBSim in Windows with mingw (Cython fails to build python bindings)
```
cmake -DCYTHON_EXECUTABLE:FILEPATH="C:/ProgramData/Anaconda3/pkgs/cython-0.29.20-py37ha925a31_0/Scripts/cython.exe" -DPYTHON_EXECUTABLE:FILEPATH="C:/Program Files/WindowsApps/PythonSoftwareFoundation.Python.3.8_3.8.1008.0_x64__qbz5n2kfra8p0/python3.8.exe" -DINSTALL_PYTHON_MODULE:BOOL="1" 

cmake ../jsbsim-repo/ -DCYTHON_EXECUTABLE="C:/ProgramData/Anaconda3/pkgs/cython-0.29.20-py37ha925a31_0/Scripts/cython.exe"

mingw32-make.exe JSBSim
mingw32-make.exe PythonJSBSim (fails here, python/setup.py doesn't have support for MinGW)
```
