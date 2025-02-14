# SRO Path Planning

## Overview
This project implements a **path planning algorithm** using **Probabilistic Roadmaps (PRM)** in MATLAB to navigate a **mobile robot in a simulated environment**. The system utilizes a **binary occupancy map** for obstacle avoidance and communicates with **CoppeliaSim (V-REP)** for real-world simulation.

## Features
- **Map-Based Path Planning**: Loads a pre-defined occupancy map (`map.png`).
- **PRM Algorithm**: Generates a roadmap and computes an optimal path.
- **Simulation Integration**: Connects with **CoppeliaSim** via its remote API.
- **Pure Pursuit Controller**: Ensures smooth trajectory following.
- **Robot Motion Control**: Implements inverse kinematics for differential drive.

## Project Structure
```
📂 project_root/
├── 📜 path_planning.m  # Main MATLAB script for path planning
├── 🗺️ map.png          # Binary occupancy grid for environment
├── 🗺️ map2.ttt         # CoppeliaSim simulation scene
└── 📝 README.md        # Project documentation
```

## Dependencies
Ensure you have the following installed:
- **MATLAB with Robotics Toolbox**
- **CoppeliaSim (V-REP)**

## Usage
### 1. Load the Map and Generate a Roadmap
```matlab
im_map = imread('map.png');
im_bin = im2bw(im_map);
myMaplogical = not(logical(im_bin));
map = binaryOccupancyMap(myMaplogical,100);
show(map);
```

### 2. Compute the Optimal Path
```matlab
PRM = mobileRobotPRM(map, 100);
show(PRM);

startPosition = [0.175 4.25];
goalPosition = [-4.358 -4.469];
path = findpath(PRM, startPosition, goalPosition);
show(PRM);
```

### 3. Connect to CoppeliaSim
```matlab
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);
```
If successful, the console will display:
```
connected to remote API server
```

### 4. Control Robot Motion
The script calculates inverse kinematics to control the **Pioneer P3-DX robot**:
```matlab
[v, w] = pp(currentPoseMatlab);
[phiR, phiL] = invkinem(v, w);
```

## Implementation Details
### Motion Control
- Uses a **Pure Pursuit Controller** to follow the planned path.
- Converts between MATLAB and **CoppeliaSim coordinate systems** using:
```matlab
function [posOut]= posconvert(trID,posIn)
```

### Inverse Kinematics
- Computes wheel velocities for the **differential drive robot**:
```matlab
function [pR,pL] = invkinem(v,w)
```
