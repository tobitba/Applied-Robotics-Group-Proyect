# general info
--------------
This is a software library for robot control called 
Simple Manipulator Control (SMC).
Its existence serves two main goals:
1. to speed up the development, testing and deployment
of model-based controllers
2. to provide a ready-to-use collection of common control strategies

These goals are achieved through the following.
The library is designed to be as simple as possible, both in terms of logic 
and the total amount of code. Its skeleton provides only a common
interface to both real and simulated robots,
and a wrapper around control loops which are to be written by the users.
It also provides the following common controllers: 
- joint space control
- inverse kinematics via:
  - jacobian pseudoinverse
  - various QP formulations, with multiple QP solvers
- impedance control
- dynamic motion primitives (DMP)
- optimal control and MPC

Note that this simplicity comes at the cost of some comfort -
if this library was a car it would be a Lada Niva,
not an S-class Mercedes. It is easy to fix or replace any part,
and it runs though all weather conditions and deep mud.
But there is no power steering, let alone seat heating or
an "infotainment system". And windows are opened manually by rotating knobs.
This is guided by a personal belief that software should get out of the way 
of the fact that a robotic manipulator is just 6+ motorized pendulums,
the control of which are easy provided adequate mathematical background.

## basic technical information
- the code is written in Python
- the code is tested on Arch Linux and Ubuntu 22 and 24, but should work on any Linux, Ubuntu wsl in Windows, and MacOS (with some exceptions)
- all of the robotics-related dependent libraries are written in C++ with Python bindings, proving fast execution despite the Python front-end
- if installing on own system install the latest python dependencies and it should work (but we do not provide guarantees)


## currently supported robots
  - UR robots (all but UR5e require a manual selection of urdf)
  - 2 flavors of mobile YuMi
  - Heron (MiR platform with UR arm)

### porting to new robots
- robot models are loaded via URDF
- an interface to the robot (socket with a protocol, ros topic,...) needs to be provided.
  The interface has to contain functions for reading of the robot's state (joint positions, velocities, f/t sensor as vectors)
  and functions to send control inputs (joint velocities). Alternatively, if the robot has a ROS2 driver, there are instructions
  for writing an SMCNode.


# contents of this repository
---------------------------------
## example usage
----------------
In the examples directory. Different arguments and script description is available through python \[script.py\] --help .
More complex examples also come with a technical report pdf which explains the math behind the code.

## the library 
--------------------------------
- smc Python package in python/smc
- robot description files in python/smc/robots/robot_descriptions : 
    - UR5e with Robotiq Hand-e gripper: urdf containing both the arm and the gripper, and other urdf-related files
- visualization in python/smc/visualization:
  - real-time plotting in plotters.py
  - visualizer (showing the robot and the workspace) in visualizer.py
- control in python/smc/control:
  - control_loop_manager - wrapper around control loops which manages visualization, plotting, logging and timing
  - controller_templates - control loop templates for point to point tasks, path following tasks
  - cartesian_space - control loops for inverse kinematics, impedance control
  - freedrive - put the robot in freedrive (leadthrough) mode - move the robot around manually - can record joint angles and poses, effectively teaching by demonstration
  - joint_space - control loops for joint space control
  - optimal_control - OCP definitions, OCP solving + trajectory following control loops, MPC control loops
  - dmp - dynamic motion primitives, control loop to follow a dmp (NOTE: bare bones,needs further development)

## documentation
------------------------
- an introductory [video series](https://www.youtube.com/playlist?list=PLKOlTxWSIaGWsaJwz6oKtgGB1Ukv_RIeP) on Linux, Python, common Python packages with followup text in [tutorials repository](https://gitlab.control.lth.se/marko-g/LPR-tutorials) 
 (note: video coverage of SMC is outdated, current state is in the pdf mentioned below)
- in-depth documentation of the library, including controller math, 
  can be found in the docs directory, pdf with everything is in docs/everything/smc.pdf
- links to and basic information about the external resources are listed in the docs directory:
    - official UR documentation on the robot's capabilities and the  ur_rtde interface
    - pinocchio
    - short instructions on installing Ubuntu and installing the library via Docker
- code is well documented and short and simple enough to simply be read to be understood
- more complex examples are documented with pdf technical reports
- additional documentation will be expanded according to demand

# Installation
---------------
The library is in the python/smc directory. Navigate to python directory and do a local pip install with "pip install -e .".
## Dependencies
-------------------------
- Linux OS, but Ubuntu wsl on also Windows works, and on MacOS the simulation parts work. You are advised to do a Docker installation if not on native Linux, details below. 
Python packages:
- pinocchio - robot math
- numpy - math
- matplotlib - plotter
- meshcat, meshcat_shapes - visualizer
- \[optional\] UR robots interface: ur_rtde
- \[optional\] yumi, mobile yumi, and heron interface: ros2
- \[optional\] ik solvers: qpsolvers, quadprog, proxsuite, ecos, pin-pink (uses quadprog under the hood)
- \[optional\] opc/mpc: crocoddyl
- \[optional\] altertive ocp: casadi (requires pinocchio > 3.0)
- \[optional\] path planning: albin's path planning repos: starworlds, star_navigation, tunnel_mpc (NOTE all these repos are copy-pasted to path_generation atm)
- \[optional\] visual servoing: opencv-python
- \[optional\] drawing: pyqt6 
- \[optional\] some of the visualization: tkinter
- \[very optional\] networking: protobuf
- \[very optional\] control of some robots: ROS2 (just rcl required, but you'll probably want to verify stuff in various GUIs also)

## Docker
---------
The library is also shipped as a Docker image. Dockerfile available in docker folder. Instruction are in docs/installation.md
and in docs/everything/smc.pdf under the installation section.


# Usage on real robots
---------------------
### UR robots
The ur_rtde interface communicates with the robot over a network. It connects to the 50002 port on the robot.
If you computer and the robot are on the same network, and you set the robot ip with the --robot-ip argument,
everything should work. Client side's (your computer) firewalls shouldn't be a problem.

The simplest way to set the network up is to:
1. Directly connect your computer and the robot with an ethernet cable.
2. Create a fixed ip connection. On the robot side:
  1. On the teaching pendant go to Setting->Network
  2. Select a manual/fixed IP connection and put 192.168.1.102. 
On your computer:
  1. Create a new Ethernet connection, and under IPv4 setting put 192.168.1.101 as the host (your) address, 255.255.255.0 under netmask
  and 192.168.1.1 as gateway (just in case). 
  2. Connect to this connection once you've created it.
3. go to examples/cartesian_space and run "python clik.py --robot-ip 192.168.1.102" and move the robot to verify that everything works.

### Mobile YuMi, Heron
TODO: describe networking and what to start, or point to those robot's documentation

# ongoing development
---------------------
Can be found in development_plan.toml.
