#!/usr/bin/env python

import os
import sys
import copy
import time

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gripper import Gripper

Q1 = [-5.357888166104452, -1.884592358266012, 2.388963222503662, -2.075622860585348, -1.5848382155047815, 0.3380693793296814]
Q2 = [-5.3574323693858545, -1.9451826254474085, 2.3004908561706543, -1.9278109709369105, -1.584311310444967, 0.3370263874530792]

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
def move_to_joint_goals():
    """This function shows an example of how to move to a series of joint goals with ROS Moveit"""
    print(group.get_current_joint_values())

    group.go(Q1)
    group.stop()
    time.sleep(2)

    print(group.get_current_joint_values())

    group.go(Q2)
    group.stop()
    time.sleep(2)

    print(group.get_current_joint_values())


