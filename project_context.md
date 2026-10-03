# Robo-Poly Project Context & Architecture Overview

This document provides a comprehensive technical overview of the **Robo-Poly** project, synthesizing information from three main sources:
1. **Project Proposal Document** (`Robo-poly Project Proposal.pdf`)
2. **Robot Control Library** (`ur_simple_control` / SMC framework)
3. **Monopoly Game Engine & AI** (`Monopoly` MATLAB package)

---

## 1. Project Goal & Overview

The objective of the **Robo-Poly** project for the *Applied Robotics* course is to build a robotic system capable of playing a physical game of Monopoly against human opponents using a **UR5e robotic arm**.

To keep the project feasible and avoid complex physical manipulation of paper currency and cards, a **mixed-reality approach** is adopted:
- **Physical Components**: Physical game board, player tokens/characters, houses, and physical dice.
- **Digital Components**: The "Bank", money transactions, property cards, and financial balances are handled digitally via a Digital Bank UI.
- **Robot Actions**: Physical dice rolling, overhead computer vision dice & token reading, move calculation, and precise physical pick-and-place of tokens and houses on board tiles.

---

## 2. Project Proposal Summary (`Robo-poly Project Proposal.pdf`)

### Hardware Requirements
- **Manipulator**: Universal Robots UR5e 6-DOF robotic arm.
- **End-Effector**: Parallel jaw gripper (e.g., Robotiq Hand-e / OnRobot 2FG) for grasping dice, tokens, and house models.
- **Vision**: Overhead web camera mounted directly above the Monopoly board workspace.
- **Props**: Physical Monopoly board, physical dice, customized or 3D-printed characters and house models.
- **Interface**: Monitor or tablet displaying the Digital Bank UI for human interaction.
- **Compute**: Main PC running control software, computer vision processing, and the game engine.

### Software Architecture (4 Pillars)
1. **Control & Kinematics (SMC / Python)**:
   - Uses the Simple Manipulator Control (SMC) library.
   - Calculates Inverse Kinematics (IK) for pick-and-place tile coordinates.
   - Plans collision-free trajectories and handles gripper open/close actuation.
2. **Computer Vision (CV / Python)**:
   - Processes overhead camera feed.
   - Detects dice roll values automatically.
   - Tracks real-time positions of board tokens/pieces.
3. **Game Engine & AI (MATLAB / State Machine)**:
   - Maintains full Monopoly game state and rules.
   - Evaluates optimal financial strategies and decisions using trained Reinforcement Learning (RL) value functions.
4. **UI & System Integration (Digital Bank)**:
   - Bridges human inputs, vision output, MATLAB game engine states, and robot execution.

### Task Distribution
- **Vision Sub-team (2 members)**: Camera calibration, dice recognition, token tracking.
- **Control Sub-team (2 members)**: SMC integration, IK solvers, trajectory generation, gripper actuation.
- **AI & Logic Sub-team (2 members)**: Monopoly game engine, digital state management, RL strategy.
- **Integration & UI Lead (1 member)**: Digital Bank GUI, cross-module IPC / communication.

### Contingency & Fallback Tiers
- **Tier 1 Simplification**: Disable Computer Vision; human players manually enter dice roll outcomes into the UI.
- **Tier 2 Simplification**: Robot acts exclusively as the "Dice Roller" and "Banker"; humans manually move physical tokens on the board.

### Main Identified Challenges & Risks
- **Polyglot System Latency**: Connecting MATLAB (Game Engine) and Python (SMC & CV) introduces IPC latency and structural complexity.
- **Computational Overhead**: Simultaneous execution of MATLAB engine, Python robot control loops, real-time CV processing, and UI requires careful resource management on the host PC.

---

## 3. Robot Control Library (`ur_simple_control` / SMC)

### Design Philosophy
The **Simple Manipulator Control (SMC)** framework is a lightweight, Python-based control library. Its core goal is to provide a bare-bones, high-performance interface to real and simulated manipulators without unnecessary software overhead ("Lada Niva" philosophy).

### Core Capabilities
- **Kinematics & Dynamics**: Built on top of **Pinocchio** (C++ backend with Python bindings) for fast rigid-body kinematic and dynamic calculations.
- **UR5e Real-Robot Control**:
  - Communicates directly via **`ur_rtde`** (Real-Time Data Exchange) over TCP/IP (port 50002).
  - Supports velocity control (`speedJ`), joint position reads (`getActualQ`), velocity reads (`getActualQd`), TCP force/torque sensor reads, and freedrive mode setting.
- **Gripper Integration**:
  - Built-in support for Robotiq Hand-e (TCP port 63352), RS485 Robotiq, and OnRobot 2FG.
  - URDF model including UR5e + Robotiq Hand-e (`ur5e_with_robotiq_hande_FIXED_PATHS.urdf`).
- **Simulated Execution**:
  - `SimulatedUR5eRobotManager` runs in simulation mode using Pinocchio and Meshcat visualizer without requiring a physical robot.
  - Enables offline testing of IK, trajectory generation, and pick-and-place logic.
- **Control Algorithms**:
  - Inverse Kinematics (CLIK) via Jacobian pseudoinverse or QP formulations (proxsuite, qpsolvers, quadprog, pin-pink).
  - Joint space control, impedance control, OCP/MPC (Crocoddyl/CasADi), and Dynamic Motion Primitives (DMP).

### Key Files in `ur_simple_control`
- [ur5e.py](file:///C:/Users/BANGHO/Documents/Lund/Applied%20Robotics/Proyect/ur_simple_control/python/smc/robots/implementations/ur5e.py): Implementation of `SimulatedUR5eRobotManager` and `RealUR5eRobotManager`.
- [control_loop_manager.py](file:///C:/Users/BANGHO/Documents/Lund/Applied%20Robotics/Proyect/ur_simple_control/python/smc/control/control_loop_manager.py): Wrapper managing execution timing, logging, visualization, and safety loops.
- [vision.py](file:///C:/Users/BANGHO/Documents/Lund/Applied%20Robotics/Proyect/ur_simple_control/python/smc/vision/vision.py): Placeholder vision module (needs implementation for Monopoly dice & token recognition).

### Current Limitations & Gaps
- **OS Dependency**: Primary target is Linux (Ubuntu/Arch) or Ubuntu WSL2 on Windows. Running natively on Windows without Linux/WSL2 may face issues with `ur_rtde` C++ bindings.
- **Vision Stub**: `smc.vision` is minimal and must be expanded or replaced with custom OpenCV/ArUco/YOLO detection scripts.
- **Workspace Coordinate Mapping**: Needs mapping functions from board tile indices (1 to 40) to 3D Cartesian coordinates $(x, y, z, \text{orientation})$ relative to the UR5e base frame.

---

## 4. Monopoly Game Engine & AI (`Monopoly`)

### Design & Package Architecture
The `Monopoly` directory contains **BOT-OPOLY**, a MATLAB package designed for Monte Carlo simulation of classic Monopoly games, Reinforcement Learning (RL) value function training, and live gameplay execution.

### Key Components
- [Monopoly.m](file:///C:/Users/BANGHO/Documents/Lund/Applied%20Robotics/Proyect/Monopoly/@Monopoly/Monopoly.m): Main object class representing the game board, tracking:
  - `board`: Table with 40 tiles, property ownership, mortgage status, house counts, and player piece locations.
  - `assets`: Table per player tracking cash, net worth, and Get-Out-of-Jail-Free cards (GOJFC).
  - Member functions: `moveToken`, `payCash`, `changeNetWorth`, `buyProperty`, `trade`, `mortgage`, `develop`, `target`.
- [getState.m](file:///C:/Users/BANGHO/Documents/Lund/Applied%20Robotics/Proyect/Monopoly/@Monopoly/getState.m): Extracts feature vectors of length $57 + 4 \times N_{\text{players}}$ representing board state for model evaluation.
- [+game/turnManager.m](file:///C:/Users/BANGHO/Documents/Lund/Applied%20Robotics/Proyect/Monopoly/+game/turnManager.m): Evolving turn logic based on random policy or $\epsilon$-greedy policy over trained value functions.
- [+game/gameManager.m](file:///C:/Users/BANGHO/Documents/Lund/Applied%20Robotics/Proyect/Monopoly/+game/gameManager.m): Executes full games up to max turns or bankruptcy.
- `Classes/`: Enumerations containing Classic Monopoly data (`Properties.m`, `Tiles.m`, `Chance.m`, `CommunityChest.m`, `Action.m`, `Transaction.m`, `Resource.m`).
- `Resources/`:
  - `tutorial.mlx`: Live script tutorial for generating Monte Carlo data and training ensemble regression tree models.
  - `interface.mlx`: Live script GUI bridging learned models with live human vs. bot gameplay.
  - `Models/`: Directory containing pre-trained `.mat` value function models.

### Target Reward Function
The reward at step $k$ for player $i$ is defined in `Monopoly.target()` as:
$$R(i) = \text{netWorth}_i - \text{mean}(\text{netWorth}_{\text{opponents}})$$
This incentivizes decisions that maximize personal net worth while diminishing opponents' net worth.

### Simplifying Assumptions in BOT-OPOLY
1. **Auctions**: Simple sequential bidding starting at property mortgage value or cash on hand.
2. **Bankruptcy**: Assets of bankrupted player go to the player with highest net worth.
3. **Houses**: Houses do not strictly require even building across color sets.
4. **Trading**: Proposes 1-for-1 property swaps (though human-offered complex trades can be evaluated).

---

## 5. System Integration Architecture

To connect all three components effectively:

```
+-----------------------------------------------------------------------+
|                            DIGITAL BANK UI                            |
|             (Central Coordinator / Flask/FastAPI or Node)             |
+-------------------+-+-------------------------------+-----------------+
                    | |                               |
                    v v                               v
+-----------------------------------+   +-------------------------------+
|      MATLAB GAME ENGINE (AI)      |   |   OPENCV VISION PIPELINE      |
| - Monopoly state machine          |   | - Dice roll detection         |
| - Value function decision-making  |   | - Board token/house tracking  |
+-------------------+---------------+   +---------------+---------------+
                    |                                   |
                    +-----------------+-----------------+
                                      |
                                      v
                        +---------------------------+
                        |  PYTHON UR5e CONTROLLER   |
                        |      (ur_simple_control)  |
                        | - Board tile Cartesian IK |
                        | - Trajectory planning     |
                        | - Gripper pick & place    |
                        +---------------------------+
```

### Options for Inter-Process Communication (IPC)
1. **MATLAB Engine API for Python**: Call MATLAB functions (`game.turnManager`, `Monopoly` methods) directly from Python.
2. **Socket / REST / ZeroMQ Gateway**: Run MATLAB script as a socket server emitting state updates and receiving player choices, while Python handles UI, vision, and UR5e control.
3. **Porting Logic to Python**: Alternatively, rewrite core Monopoly state logic in Python if MATLAB licensing/IPC latency becomes a bottleneck.

---

## Summary of Next Steps for the Team
1. **Kinematics & Board Mapping**: Define 3D transformation matrices from Monopoly board tile indices $(1 \dots 40)$ to UR5e base frame coordinates.
2. **Vision Pipeline Development**: Implement dice recognition (contour/dot counting or ArUco markers) and token tracking.
3. **Bridge MATLAB & Python**: Establish communication between BOT-OPOLY MATLAB engine and Python SMC runner.
4. **Testing in Simulation**: Validate full loop (Dice roll -> AI turn decision -> IK calculation -> simulated Meshcat execution) using `SimulatedUR5eRobotManager`.
