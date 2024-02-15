# Description
Assignments for advanced robotics module in University of Birmingham (forward kinematics & inverse kinematics) 

## Requirements
 - ROS Noetic or later
 - Ubuntu 20.04 or later
 - Panda simulator package [panda_simulator](https://github.com/justagist/panda_simulator/tree/noetic-devel).

## Installation:
1. Create a new workspace:

```shell
mkdir -p ~/uob_ar_assignment/src
cd ~/uob_ar_assignment/src
catkin_init_workspace
```

2. Clone the package into the workspace:

```shell
git clone https://github.com/HyPAIR/Advanced-Robotics-Assignment.git
```

3. Build the workspace:

```shell
cd ~/uob_ar_assignment
catkin_make_isolated
source devel_isolated/setup.bash
```

## Usage

### Test forward kinematics
Launch the node to visualize [Franka Emika](https://franka.de/):

```shell
cd ~/uob_ar_assignment
source devel_isolated/setup.bash
roslaunch advance_robotics_assignment assignment1_fk.launch 
```
Open a new terminal and run the following file to visualize the results of your forward kinematics calculations.
```shell
source devel_isolated/setup.bash
cd uob_ar_assignment/src/Advanced-Robotics-Assignment/advance_robotics_assignment/assignment/assignment1_FK
python3 forward_kinematics.py 
```
![fk](https://github.com/HyPAIR/Advanced-Robotics-Assignment/blob/main/figure/fk.gif)

### Test inverse kinematics
Launch the node to visualize [Franka Emika](https://franka.de/):

```shell
cd ~/uob_ar_assignment
source devel_isolated/setup.bash
roslaunch advance_robotics_assignment assignment2_lk.launch 
```
Open a new terminal and run the following file to visualize the results of your inverse kinematics solution.
```shell
source devel_isolated/setup.bash
cd uob_ar_assignment/src/Advanced-Robotics-Assignment/advance_robotics_assignment/assignment/assignment2_IK
python3 inverse_kinematics.py 
```
![ik](https://github.com/HyPAIR/Advanced-Robotics-Assignment/blob/main/figure/ik.gif)
