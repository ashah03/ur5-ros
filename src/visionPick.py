#!/usr/bin/env python
"""
This script shows a pick and place example based on vision.

The UR5 is first moved to a specific pose (without MoveIt) where it takes pictures from. OpenCV is used to identify the location of the
object (in this case, a cork). Then, MoveIt is used to make adjustments in the x and y directions until the cork is
centered in the camera frame. Once aligned, the robot grabs the cork.

There are some adjustments that were empirically found to increase the accuracy of the system. This is to get around
some of MoveIt's unpredictability.
"""
import os
import sys
import copy
import time
from image_utils import get_cork_centroid, get_image
from math import copysign
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gripper import Gripper
import roslib;
from gripper import Gripper

roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

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


Q1 = [-5.841471854840414, -1.880625073109762, 1.967238426208496, -2.1730979124652308, -1.353576962147848, -0.2975104490863245]
#Q2 = [-5.793725941573278, -1.6985562483416956, 2.245114803314209, -2.140933338795797, -1.5324457327472132, -0.2870977560626429]
#Q2 = [-5.797215227280752, -1.790765110646383, 2.2951724529266357, -2.1428964773761194, -1.5193117300616663, -0.28790074983705694]
#Q2 = [-5.773678247128622, -1.7172182242022913, 2.291447162628174, -2.1666935125934046, -1.5579717795001429, -0.2839697043048304]
#Q2 = [-5.80492484966387, -1.7727258841144007, 2.351362943649292, -2.23500901857485, -1.5296443144427698, -0.3075040022479456]
Q2 = [-5.802346740161077, -1.727706257496969, 2.3336830139160156, -2.251301113759176, -1.537558380757467, -0.30245906511415654]
#Q2 = (-5.790848229323522, -1.6641319433795374, 2.2818641662597656, -2.133308235798971, -1.57269794145693, -0.289194409047262)
#Q2 = [-5.807982299719946, -1.7263277212726038, 2.412261724472046, -2.3110459486590784, -1.5310810248004358, -0.2967436949359339]

Q3 = [i-j for i, j in zip(Q1, Q2)]

Q5 = [-5.233350698147909, -2.263735596333639, 2.0414814949035645, -1.8042591253863733, -1.5754278341876429, -1.077151123677389]
Q6 = [-3.9884613196002405, -1.7695258299456995, 1.7130355834960938, -1.5220916906939905, -1.5590489546405237, -1.6418374220477503]

# moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                 anonymous=True)

grip = Gripper()
grip.open()
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
# group_name = "manipulator"
# group = moveit_commander.MoveGroupCommander(group_name)
# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
#                                                moveit_msgs.msg.DisplayTrajectory,
#                                                queue_size=20)

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()

try:
   wait = int(raw_input("How long should it wait..."))
except:
   wait = 20


g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
joint_states = rospy.wait_for_message("joint_states", JointState)
joints_pos = joint_states.position
g.trajectory.joint_names = JOINT_NAMES
g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0))]
g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(wait)))
client.send_goal(g)
print("waiting for finish")
client.wait_for_result()
print("finished")

time.sleep(0.5)

centroid = get_cork_centroid(get_image())
#x_adjust = (308-centroid[0])/3
x_adjust = 0
x_original = centroid[0]
x_goal = 308+x_adjust
x_error = centroid[0] - x_goal
print("x_goal is {0}".format(x_goal))
while abs(x_error) > 1:
    print(centroid)
    pose_goal = group.get_current_pose().pose
    #pose_goal.position.x -= 0.005
    #pose_goal.position.y += 0.005
    #x_delta = x_error/3500.0
    x_delta = copysign(0.003, x_error)  + x_error/4500.0
    #x_delta = copysign(0.002, x_error) + x_error/5000.0
    print(x_delta)
    pose_goal.position.x -= x_delta
    pose_goal.position.y += x_delta
    group.set_pose_target(pose_goal)
    group.go()
    #time.sleep(2)
    group.stop()
    group.clear_pose_targets()
    centroid = get_cork_centroid(get_image())
    x_error = centroid[0] - x_goal

centroid = get_cork_centroid(get_image())
y_adjust = (221-centroid[1])/8
y_goal = 221+y_adjust
y_error = centroid[1] - y_goal
z_error = (centroid[1]-221)/1000
while abs(y_error) > 1:
    print(centroid)
    pose_goal = group.get_current_pose().pose
    #pose_goal.position.x -= 0.005
    #pose_goal.position.y += 0.005
    y_delta = copysign(0.003, y_error) + y_error/4500.0
    #y_delta = copysign(0.002, y_error) + y_error/5000.0
    #y_delta = y_error/1400.0
    print(y_delta)
    pose_goal.position.x += y_delta
    pose_goal.position.y += y_delta
    group.set_pose_target(pose_goal)
    group.go()
    #time.sleep(2)
    group.stop()
    group.clear_pose_targets()
    centroid = get_cork_centroid(get_image())
    y_error = centroid[1] - y_goal
print(get_cork_centroid(get_image()))

#
# pose_goal = group.get_current_pose().pose
# pose_goal.position.z += z_error
# group.set_pose_target(pose_goal)
# group.go()
# group.stop()
# group.clear_pose_targets()


g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
joint_states = rospy.wait_for_message("joint_states", JointState)
joints_pos = joint_states.position

Q4 = [i-j for i, j in zip(joints_pos, Q3)]
if x_original > 308:
    Q4[4] = (Q2[4] + Q4[4])/2
# Q4[4] = Q2[4]

g.trajectory.joint_names = JOINT_NAMES
g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0))]
g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(5)))
print("sending...")
client.send_goal(g)
print("waiting for finish")
client.wait_for_result()
print("finished")



g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
joint_states = rospy.wait_for_message("joint_states", JointState)
joints_pos = joint_states.position
g.trajectory.joint_names = JOINT_NAMES
g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0))]
g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(1)))

print("sending...")
client.send_goal(g)
print("waiting for finish")
client.wait_for_result()
print("finished")

grip.close()

# action = raw_input("Continue?")
# if action != '':
#     grip.open()
#     sys.exit(0)

g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
joint_states = rospy.wait_for_message("joint_states", JointState)
joints_pos = joint_states.position
g.trajectory.joint_names = JOINT_NAMES
g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0))]
g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q5, velocities=[0]*6, time_from_start=rospy.Duration(10)))
g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q6, velocities=[0]*6, time_from_start=rospy.Duration(20)))


print("sending...")
client.send_goal(g)
print("waiting for finish")
client.wait_for_result()
print("finished")

grip.open()

print rospy.wait_for_message("joint_states", JointState).position

