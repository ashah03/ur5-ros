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


def move_to_pos():
    """ This function shows an example of how to move the UR5 to a position with MoveIt"""
    pose_goal = group.get_current_pose().pose
    print("current location:")
    print(pose_goal)
    pose_goal.position.x = 0.01
    pose_goal.position.y = 0.46
    pose_goal.position.z = 0.3
    print("goal:")
    print(pose_goal)
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    #time.sleep(2)
    group.stop()
    group.clear_pose_targets()
    print(group.get_current_pose().pose)


def follow_cartesian_path():
    """ This function shows an example of how to have the UR5 follow a cartesian path with MoveIt"""
    waypoints = []

    pose_goal = group.get_current_pose().pose
    pose_goal.position.x = 0.227
    pose_goal.position.y = 0.296
    pose_goal.position.z = 0.28  # 0.21

    waypoints.append(copy.deepcopy(pose_goal))

    pose_goal = group.get_current_pose().pose
    pose_goal.position.x = 0.271
    pose_goal.position.y = 0.299
    pose_goal.position.z = 0.22  # 0.21

    waypoints.append(copy.deepcopy(pose_goal))

    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

    print(waypoints)

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)
    time.sleep(2)
    group.execute(plan)
