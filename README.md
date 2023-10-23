# Teleoperation for UR5

## Steps to setup the environment

cd ur5_ws
roslaunch ur_gazebo ur5_bringup.launch
change gravity to 0
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
roslaunch ur5_moveit_config moveit_rviz.launch

rosrun controller_manager controller_manager list


change controller:

rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_position_controller']
stop_controllers: ['eff_joint_traj_controller']
strictness: 0
start_asap: false
timeout: 0.0"

roslaunch icl_ur5_setup_moveit_config moveit_servo.launch 
roslaunch ur5_joystick_control ur5_joy.launch