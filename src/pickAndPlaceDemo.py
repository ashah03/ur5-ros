#!/usr/bin/env python

"""
This script shows a pick and place example for fixed locations

MoveIt is not being used here, but joint goals are being directly sent to the correct ROS topic. This is because MoveIt
does not move to positions or joints very accurately, so we are using the underlying interface instead.
"""

import time
import roslib;
from gripper import Gripper

roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

Q1 = [-5.357888166104452, -1.884592358266012, 2.388963222503662, -2.075622860585348, -1.5848382155047815, 0.3380693793296814]
Q2 = [-5.3574323693858545, -1.9451826254474085, 2.3004908561706543, -1.9278109709369105, -1.584311310444967, 0.3370263874530792]
Q3 = [-5.743474606667654, -1.7474830786334437, 2.167590618133545, -2.0011423269854944, -1.5819767157184046, -0.04833347002138311]
Q4 = [-5.741568330918447, -1.693533722554342, 2.239975929260254, -2.1274550596820276, -1.5824559370623987, -0.04512149492372686]

rospy.init_node("mover", anonymous = True)
grip = Gripper()
grip.open()

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()
g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
joint_states = rospy.wait_for_message("joint_states", JointState)
joints_pos = joint_states.position
g.trajectory.joint_names = JOINT_NAMES
g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
joints_pos = list(joints_pos)
joints_pos[0] = -4.9
g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(5.0)))

g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(7.0)))
client.send_goal(g)
print("waiting for finish")
client.wait_for_result()
print("finished")

print(rospy.wait_for_message("joint_states", JointState).position)
grip.close()

g.trajectory.points = [JointTrajectoryPoint(positions=rospy.wait_for_message("joint_states", JointState).position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]

g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(2.0)))

g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(7.0)))

g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(9.0)))

client.send_goal(g)
print("waiting for finish")
client.wait_for_result()
print("finished")

#grip = Gripper()
grip.open()

g.trajectory.points = [JointTrajectoryPoint(positions=rospy.wait_for_message("joint_states", JointState).position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]


g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(2.0)))

client.send_goal(g)
print("waiting for finish")
client.wait_for_result()
print("finished")