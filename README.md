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
### Flightgear
```
sudo apt-get install flightgear
```

### JSBSim (Build from source)
```
sudo apt-get install cmake make g++ python3 cython3 python3-setuptools

mkdir Sim; cd Sim; mkdir JSBSim; cd JSBSim
sudo apt-get install git
git clone https://github.com/JSBSim-Team/jsbsim.git jsbsim-repo
mkdir build; cd build
cmake ../jsbsim-repo/ -DCYTHON_EXECUTABLE=/usr/bin/cython3 -DINSTALL_PYTHON_MODULE=ON
make JSBSim
make PythonJSBSim
make install
make test
```

### RAPTRS (Minimal for compiling SOC code for AMD64)

```
sudo apt-get install g++ libEigen3-dev

cd Goldy3
git clone https://github.com/UASLab/RAPTRS.git
cd RAPTRS
git checkout SimOverhaul

cd software;
make flight_amd64
make datalog_amd64
```

### OpenFlightSim
```
sudo apt-get install socat netcat python3-pygame python3-serial python3-numpy python3-pandas

mkdir Goldy3; cd Goldy3
git clone https://github.com/UASLab/OpenFlightSim.git
cd OpenFlightSim
git checkout SimOverhaul
cd Simulation
```

Tests 1:
Start FlightGear from a Terminal (Goldy3/OpenFlightSim/Simulation):

```./fgfs_JSBSim UltraStick25e```

then from another Terminal (Goldy3/OpenFlightSim/Simulation):

```JSBSim scripts/jsb_UltraStick25e_Cruise.xml```
(should run for 100 seconds)

Test 2:
Start FlightGear again, then from another Terminal (Goldy3/OpenFlightSim/Simulation):

```python3 python/JSBSim_Script_Demo.py```
(run for 10 seconds)

### Configs
```
cd ~/Goldy3;
git clone https://github.umn.edu/UAV-Lab/Config.git
```

### Tests
Software in the Loop Test:
Using multiple terminals, all at: Goldy/OpenFligtSim/Simulation

```./start_SIL_Comm.sh``` (this can stay running)

```./fgfs_JSBSim.sh UltraStick25e``` (needs to restart each session)

```python3 python/JSBSim_SIL_Demo.py```

```~/Goldy3/RAPTRS/software/bin/flight_amd64 ~/Goldy3/Config/thor.json```

(This should start a SIL, use a connected joystick to fly)

##  Windows 10 with Windows Linux Subsystem - (Debian 10.4)
Install github desktop (https://desktop.github.com/)

Clone: https://github.com/UASLab/OpenFlightSim.git to {path to ...}/Goldy3/OpenFlightSim

Clone: https://github.com/UASLab/RAPTRS.git to {path to ...}/Goldy3/RAPTRS

Clone: https://github.umn.edu/UAV-Lab/Config.git to {path to ...}/Goldy3/Config

Install WLS2 and Debian (https://docs.microsoft.com/en-us/windows/wsl/install-win10)


In a WLS2-Debian Console, make a link to the Windows folder:

```ln -s /mnt/{path to ...}/Goldy3/ Goldy3```


OpenFlightSim uses the JSBSim Python bindings. First, get Python3 installed using conda as the Python Package Manager.

Install Miniconda: 
https://repo.anaconda.com/miniconda/Miniconda3-py38_4.8.3-Windows-x86_64.exe

Then open a "Anaconda Prompt" in Windows.

```
conda install numpy scipy matplotlib ipython jupyter pandas sympy nose h5py spyder pyserial
conda install -c conda-forge slycot control
pip install pygame
```

### Flightgear
Install FlightGear in Windows10. (https://www.flightgear.org/download/) Tested with version 2018.3.5.

Add FlightGear{version}/bin to the Windows Environment Path...

### JSBSim in Windows
Use JSBSim release: https://github.com/JSBSim-Team/jsbsim/releases

Open a "Anaconda Prompt" in Windows:

```
pip install jsbsim --no-index -f "https://github.com/JSBSim-Team/jsbsim/releases/download/Windows-MSVC/JSBSim-1.1.0.dev1-50-cp38-cp38-win_amd64.whl"
```

### RAPTRS in WSL2-Debian (minimal for compiling SOC code for AMD64)
```
sudo apt-get install g++ libEigen3-dev

cd Goldy3/RAPTRS/software;
make flight_amd64
make datalog_amd64
```

## OpenFlightSim
Test 1:
Start FGFS in Windows with: fgfs_JSBSim.bat
then in a "Anaconda Prompt":

```JSBSim scripts/jsb_UltraStick25e_Cruise.xml```

(should run for 100 seconds)

Test 2:
Start FGFS in Windows with: fgfs_JSBSim.bat
then in a "Anaconda Prompt":

```python3 python/JSBSim_Script_Demo.py```

(should run for 10 seconds)

### Integrated Tests
Software in the Loop Test:
Start FGFS in Windows with: fgfs_JSBSim.bat

In a WSL-Debian Console (at folder: Goldy3/OpenFlightSim/Simulation):

```./start_SIL_Comm.sh``` (this can stay running)

Anaconda Prompt (at folder: {path to ...}/Goldy3/OpenFlightSim/Simulation):

```python python\JSBSim_SIL_Demo.py```

In another WSL-Debian Console (at folder: Goldy3/OpenFlightSim/Simulation):

```~/Goldy3/RAPTRS/software/bin/flight_amd64 ~/Goldy3/Config/thor.json```

(This should start a SIL, use a connected joystick to fly)
