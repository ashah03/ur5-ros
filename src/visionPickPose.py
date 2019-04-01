#!/usr/bin/env python
"""
This script shows a pick and place example based on vision.

The UR5 is first moved to a specific pose (without MoveIt) where it takes pictures from. OpenCV is used to identify the
location of the object (in this case, a cork). The offsets are calculated based on the pixel value of the cork, and the
robot then moves to the correct location and grips the cork.

This class uses pose goals, as opposed to joint goals in visionPick.py
"""

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
from image_utils import get_image, get_cork_centroid



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

g = Gripper()
g.open()


pose_goal = group.get_current_pose().pose
print("current location:")
print(pose_goal)
pose_goal.position.x = 0.29145
pose_goal.position.y = 0.27931
pose_goal.position.z = 0.43675
pose_goal.orientation.x = -0.17844
pose_goal.orientation.y = 0.45305
pose_goal.orientation.z = 0.35205
pose_goal.orientation.w = 0.79936

move_pixel = 0.00036

print("goal:")
print(pose_goal)
group.set_pose_target(pose_goal)
group.go(wait=True)
#time.sleep(2)
group.stop()
group.go(wait=True)
group.stop()
group.clear_pose_targets()
print(group.get_current_pose().pose)

time.sleep(3)

print("goal:")
print(pose_goal)
group.set_pose_target(pose_goal)
group.go(wait=True)
#time.sleep(2)
group.stop()
group.go(wait=True)
group.stop()
group.clear_pose_targets()
print(group.get_current_pose().pose)

time.sleep(1)

centroid = get_cork_centroid(get_image())
x_error = centroid[0] - 308
x_offset = x_error * move_pixel
y_error = centroid[1] - 221
y_offset = y_error * move_pixel

pose_goal = group.get_current_pose().pose
pose_goal.position.x = 0.28936 + y_offset - x_offset
pose_goal.position.y = 0.30388 + y_offset + x_offset
pose_goal.position.z = 0.23
pose_goal.orientation.x = -0.26157
pose_goal.orientation.y = 0.66279
pose_goal.orientation.z = 0.28383
pose_goal.orientation.w = 0.64167
group.set_pose_target(pose_goal)
group.go(wait=True)
group.stop()
print(pose_goal)

g.close()


pose_goal = group.get_current_pose().pose
pose_goal.position.x = -0.035
pose_goal.position.y = 0.3
pose_goal.position.z = 0.482
pose_goal.orientation.x = -0.362
pose_goal.orientation.y = 0.308
pose_goal.orientation.z = 0.628
pose_goal.orientation.w = 0.616
group.set_pose_target(pose_goal)
group.go(wait=True)
group.stop()
group.clear_pose_targets()
print(pose_goal)

time.sleep(1)

pose_goal = group.get_current_pose().pose
pose_goal.position.x = -0.289
pose_goal.position.y = 0.364
pose_goal.position.z = 0.3935
pose_goal.orientation.x = 0.6565
pose_goal.orientation.y = 0.2771
pose_goal.orientation.z = -0.6243
pose_goal.orientation.w = 0.3199
group.set_pose_target(pose_goal)
group.go(wait=True)
group.stop()
print(pose_goal)

time.sleep(1)
g.open()
time.sleep(1)

#
# centroid = get_cork_centroid(get_image())
# x_error = centroid[0] - 308
# while abs(x_error) > 1:
#     print(x_error)
#     centroid = get_cork_centroid(get_image())
#     x_error = centroid[0] - 308
#     pose_goal = group.get_current_pose().pose
#     pose_goal.position.x -= move_pixel * x_error
#     pose_goal.position.y += move_pixel * x_error
#     group.set_pose_target(pose_goal)
#     group.go(wait=True)
#     group.stop()
#     group.go(wait=True)
#     group.stop()
#     group.clear_pose_targets()
#
#
#
# centroid = get_cork_centroid(get_image())
# y_error = centroid[1] - 221
# while abs(y_error) > 1:
#     print(y_error)
#     centroid = get_cork_centroid(get_image())
#     pose_goal = group.get_current_pose().pose
#     y_error = centroid[1] - 221
#     pose_goal.position.x += move_pixel * y_error
#     pose_goal.position.y += move_pixel * y_error
#     group.set_pose_target(pose_goal)
#     group.go(wait=True)
#     group.stop()
#     group.go(wait=True)
#     group.stop()
#     group.clear_pose_targets()
#
#
# pose_goal = group.get_current_pose().pose
# pose_goal.position.x -= 0.01340
# pose_goal.position.y -= 0.00628
# pose_goal.position.z -= 0.22446
# pose_goal.orientation.x = -0.25023
# pose_goal.orientation.y = 0.63984
# pose_goal.orientation.z = 0.29415
# pose_goal.orientation.w = 0.66442
#
# print(pose_goal)
# group.set_pose_target(pose_goal)
# group.go(wait=True)
# group.stop()
# time.sleep(2)
# group.go(wait=True)
# group.stop()
# group.clear_pose_targets()