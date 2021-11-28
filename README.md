# anticollision-trajectory

This repository was developed in partial completition of the Fall 2021 ME7752 course at The Ohio State University.


## Usage
These instructions assume you have pulled all files to a ROS workspace and sucessfully compiled them. Compiler requirements are detailed in each package.


#### 1 - Launch Robot Support Packages (via linux terminal)

Change `robot_ip` to match the local IP address of the industrial robot controller.
```Bash
roslaunch anticollision_trajectory gp7_controllers.launch robot_ip:=192.168.1.100
```


#### 2 - Matlab Scripts (via Matlab GUI or headless)

Assumes ROS Master is listening on default port, `11311`
```Matlab
rosinit('http://localhost:11311')
```

Check ROS Environment Variables:
The URI should give the location of the actively running ROS Master instance.
Hostname and IP (precedence given to hostname) indicates YOUR current network location.
```Matlab
getenv('ROS_MASTER_URI')
getenv('ROS_HOSTNAME')
getenv('ROS_IP')
```

Execute Matlab Scripts
```Matlab
MechFinalProject_DEV.m
```


## Requirements

**Designed for:**
* Ubuntu 20.04 LTS
* ROS 1 - Melodic OR Noetic
* MATLAB 2021b

**Matlab Software and Toolboxes:**
* Matlab 2021b
* [Robotic System Toolbox (R) Matlab](https://www.mathworks.com/products/robotics.html)
* Peter Corke's [Robotics Toolbox for Matlab](https://github.com/petercorke/robotics-toolbox-matlab) _... EOL Jan 2022_
* Peter Corke's [Spatial Math Toolbox](https://github.com/petercorke/spatialmath-matlab)  _... EOL Jan 2022_
* [Simulink](https://www.mathworks.com/products/simulink.html)
* [Simscape](https://www.mathworks.com/products/simscape.html)

