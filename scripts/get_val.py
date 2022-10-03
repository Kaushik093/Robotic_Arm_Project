#!/usr/bin/env python

import rospy
import moveit_commander
import math

joint_angles=[]


group=moveit_commander.MoveGroupCommander("manipulator")
rospy.init_node("angles_pos",anonymous=True)

# print("joint angles : ",group.get_current_joint_values())

for i in range(len(group.get_current_joint_values())):
    joint_angles.append(group.get_current_joint_values()[i]*180/math.pi)
print("joint angles in degrees : ",joint_angles)
   
# print("joint angles in degrees : ",joint_angles)
print("Pose of end effector : ",group.get_current_pose())