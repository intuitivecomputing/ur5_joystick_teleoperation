#!/usr/bin/env python
import rospy, moveit_commander,roslaunch, math
import moveit_msgs.msg, geometry_msgs.msg
import time, actionlib,sys,tf, tf2_ros, copy
import numpy as np
from numpy import linalg as LA
from tf import TransformListener
from std_msgs.msg import String, Bool, Float64
import geometry_msgs.msg, control_msgs
from actionlib import SimpleActionClient, GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandActionGoal
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from controller_manager_msgs.srv import SwitchController


class JoystickUR5Control:
    def __init__(self):
        rospy.init_node('joystick_ur5_control', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.ee_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        self.gripper_pub = rospy.Publisher('/icl_gripper/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)  # Gripper control topic

        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.rate = rospy.Rate(10)  # 10hz
        self.gripper_pressed = False
        self.translation_mode = True
        self.prev_button_state_A = 0  # Assuming the initial state of the button is 0
        self.prev_button_state_B = 0  # Assuming the initial state of the button is 0
        self.last_mode_change_time = rospy.Time.now()
        self.vel_scale = 0.5
        self.rot_scale = 1

        self.start_controller=['joint_group_position_controller']
        self.stop_controller=['eff_joint_traj_controller']
        # self.change_controller(self.start_controller, self.stop_controller)


    def handle_button_press_B(self, data):
        # Assuming data.buttons is a list or array
        current_button_state = data
        current_time = rospy.Time.now()
        # print(current_button_state)
        if current_button_state == 1 and self.prev_button_state_B == 0 and (current_time - self.last_mode_change_time > rospy.Duration(1)):
            self.prev_button_state_B = current_button_state
            self.last_mode_change_time = current_time
            return True
        else:
            self.prev_button_state_B = current_button_state
            return False

    def handle_button_press_A(self, data):
        # Assuming data.buttons is a list or array
        current_button_state = data
        current_time = rospy.Time.now()
        # print(current_button_state)
        if current_button_state == 1 and self.prev_button_state_A == 0 and (current_time - self.last_mode_change_time > rospy.Duration(1)):
            self.prev_button_state_A = current_button_state
            self.last_mode_change_time = current_time
            return True
        else:
            self.prev_button_state_A = current_button_state
            return False

    def joy_callback(self, data):

        if(not self.gripper_pressed and data.axes[5] != 0):
            # print(data.axes[5])
            self.gripper_pressed = True
        gripper_command = GripperCommandActionGoal()
        gripper_command.goal.command.position = 0.4 * (1 - data.axes[5])  # Map joystick input to gripper position
        if self.gripper_pressed:
            self.gripper_pub.publish(gripper_command)

        # press A for auto homing 
        if (self.handle_button_press_A(data.buttons[0])):
            self.move_to_home_position()

        # press B for mode switching
        if (self.handle_button_press_B(data.buttons[1])):
            self.translation_mode = not self.translation_mode
            print("Translation mode:", self.translation_mode)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        if(self.translation_mode):
            twist_msg.twist.linear.x = self.vel_scale* data.axes[1]  # Left stick up and down for controlling the x-axis
            twist_msg.twist.linear.y = self.vel_scale* data.axes[0]  # Left stick left and right for controlling the y-axis
            twist_msg.twist.linear.z = self.vel_scale* data.axes[4]  # Right stick up and down for controlling the z-axis
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
        else:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = self.rot_scale* data.axes[0]
            twist_msg.twist.angular.y = self.rot_scale* data.axes[1]
            twist_msg.twist.angular.z = self.rot_scale* data.axes[4]
        # end_effector_command = "speedl([{},{},{}], 0.1, 0.1)".format(twist.linear.x, twist.linear.y, twist.linear.z)  # Adjust the speed values as needed
        self.ee_pub.publish(twist_msg)

    def change_controller(self, start_controller, stop_controller):
        timeout_value = 0.5
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            rospy.set_param('/controller_manager/incoming_command_timeout', timeout_value)
            switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            response = switch_controller(
                start_controllers=start_controller,
                stop_controllers=stop_controller,
                strictness=0,
                start_asap=False,
                timeout=0.1
            )
            # print("Service call successful:", response.ok)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def move_to_home_position(self):

        self.change_controller(self.stop_controller, self.start_controller)
        print("changed controller to eff")
        rospy.sleep(1)
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)
        # self.open_gripper()
        joint_goal=[-0.43066150346864873, -1.5803573767291468, 1.5510215759277344, -1.6357649008380335, -1.5546439329730433, 0.07729113847017288]

        # [-0.4587190786944788, -1.599515740071432, 1.7295589447021484, -0.1305630842791956, 1.8112632036209106, -0.0008500258075159195]
        self.group.go(joint_goal,wait=True)
        self.group.stop()
        print("moved to home")
        self.change_controller(self.start_controller, self.stop_controller)
        rospy.sleep(1)
        print("changed controller to pos")


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    joystick_ur5_control = JoystickUR5Control()
    joystick_ur5_control.run()
