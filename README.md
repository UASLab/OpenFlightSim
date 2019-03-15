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
