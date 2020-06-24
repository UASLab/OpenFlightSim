# OpenFlightSim
Open Flight Simulation (oFS)

www.uav.aem.umn.edu


oFS is a companion to OpenFlightAnalysis for data analysis and RAPTRS for flight software.

https://github.com/UASLab/OpenFlightAnalysis

https://github.com/bolderflight/RAPTRS


oFS is intended as a tool to generate and simulate flight vehicles. Currently, the tools use OpenVSP and/or AVL as sources for Aerodynamic data. The aerodynamic data is parsed and augmented with sub-system specification (propulsion, actuation, etc.) into an intermediate format known as oFDM (Open Flight Dynamics Model). The oFDM can be used to generate a complete simulation definition for JSBSim (or Flightgear) to make a flyable aircraft model. JSBSim models are augmented with addition I/O capability to allow for both software-in-the-loop (SIL) and hardware-in-the-loop (HIL) testing with RAPTRS flight systems.

http://web.mit.edu/drela/Public/web/avl/

http://openvsp.org/

https://github.com/JSBSim-Team/jsbsim

http://home.flightgear.org/download/

# Install
## Linux (Debian 10.4):
### JSBSim
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

### Flightgear
sudo apt-get install flightgear

### RAPTRS (minimal for compiling SOC code for AMD64)
cd Goldy3;
git clone https://github.com/UASLab/RAPTRS.git
cd RAPTRS
git checkout SimOverhaul

sudo apt-get install g++ libEigen3-dev

cd software;
make flight_amd64
make datalog_amd64

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
(start FGFS on windows, then in WSL2)
JSBSim scripts/jsb_UltraStick25e_Cruise.xml (should run for 100 seconds)
- or -
python3 python/JSBSim_Script_Demo.py  (run without error for 10 seconds, displays a 10)

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

Simple Joystick Test:
python3 python/Joystick_Demo.py  (FAILS!!)

SIL TEST
(start FGFS on windows, then in multiple WSL2 terminals)
./start_SIL_Comm.sh
python3 python/JSBSim_SIL_Demo.py 
~/Goldy3/RAPTRS/software/bin/flight_amd64 ~/Goldy3/Config/thor.json
(FAILS!! - joystick issue)

##  Windows 10 with Windows Linux Subsystem - (Debian 10.4)
First, install WLS2 and Debian (https://docs.microsoft.com/en-us/windows/wsl/install-win10)

## JSBSim with WLS2-Debian
(same as Linux install)

## JSBSim in Windows with mingw (Cython fails to build python bindings)
```
cmake -DCYTHON_EXECUTABLE:FILEPATH="C:/ProgramData/Anaconda3/pkgs/cython-0.29.20-py37ha925a31_0/Scripts/cython.exe" -DPYTHON_EXECUTABLE:FILEPATH="C:/Program Files/WindowsApps/PythonSoftwareFoundation.Python.3.8_3.8.1008.0_x64__qbz5n2kfra8p0/python3.8.exe" -DINSTALL_PYTHON_MODULE:BOOL="1" 

mingw32-make.exe JSBSim
mingw32-make.exe PythonJSBSim (fails here)
```

### Flightgear
Install FlightGear in Windows10 (not WSL2). (https://www.flightgear.org/download/) Tested with version 2018.3.5.

