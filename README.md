# Teleoperation for UR5

## Prerequisite
1. git clone universal_robot ROS package using `git clone https://github.com/ros-industrial/universal_robot.git`
2. install moveit_servo using `sudo apt install ros-melodic-moveit-servo`

## Steps to setup the environment
```
cd ur5_ws
roslaunch ur_gazebo ur5_bringup.launch
```
change gravity to 0 in the gazebo gui

```
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
roslaunch ur5_moveit_config moveit_rviz.launch
```
use the following command to list currently available controllers:

```
rosrun controller_manager controller_manager list
```

In order for the joystick to work, we have to change the controller from __eff_joint_traj_controller__ to __joint_group_position_controller__ using:
```
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_position_controller']
stop_controllers: ['eff_joint_traj_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```
After everything is set, launch the following packages to start joystick teleoperation:

```
roslaunch icl_ur5_setup_moveit_config moveit_servo.launch 
roslaunch ur5_joystick_control ur5_joy.launch
```
