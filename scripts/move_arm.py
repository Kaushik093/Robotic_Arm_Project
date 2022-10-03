#! /usr/bin/env python3

import rospy
import sys
import math
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


rospy.init_node("move_group_python",anonymous=True)
group=moveit_commander.MoveGroupCommander("manipulator")


pose_target= geometry_msgs.msg.Pose()
joint_angles=[]
# Setting target pose for end effector

# q = quaternion_from_euler(0,0,0) 
pose_target.orientation.x= 0.6025924555050339
pose_target.orientation.y= -0.3248326766835255
pose_target.orientation.z= 0.2933271227926523
pose_target.orientation.w = 0.667326954169

pose_target.position.x = 0.20754018659960138
pose_target.position.y = 2.271022464834049e-06
pose_target.position.z = 0.3765736583802459


group.set_pose_target(pose_target)

group.set_planner_id("RRTConnectkConfigDefault")

plan=group.plan()
group.go(wait=True)   # Execute the plan and wait for completion

group.stop()      #Stop the robot

group.clear_pose_targets()   #Clear the target

moveit_commander.roscpp_shutdown()

# for i in range(len(group.get_current_joint_values())):
#     joint_angles.append(group.get_current_joint_values()[i]*180/math.pi)
# print("joint angles in degrees : ",joint_angles)

# Move it cmdline interface : rosrun moveit_commander moveit_commander_cmdline.py