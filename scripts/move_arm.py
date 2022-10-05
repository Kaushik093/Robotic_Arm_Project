#! /usr/bin/env python3

import rospy
import sys
import math
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler

# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()


rospy.init_node("move_group_python",anonymous=True)
group=moveit_commander.MoveGroupCommander("manipulator")


pose_target= geometry_msgs.msg.Pose()
joint_angles=[]
# Setting target pose for end effector

q = quaternion_from_euler(0,0,0) 

pose_target.orientation.x= q[0]
pose_target.orientation.y= q[1]
pose_target.orientation.z= q[2]
pose_target.orientation.w = q[3]

pose_target.position.x = 0.20754018659960138
pose_target.position.y = 2.271022464834049e-06
pose_target.position.z = 0.3765736583802459

# group.set_velocity_scaling_factor(0.1)
group.set_pose_target(pose_target)

plan=group.plan()
group.go(wait=True)   # Execute the plan and wait for completion

group.stop()      #Stop the robot

for i in range(len(group.get_current_joint_values())):
    joint_angles.append(group.get_current_joint_values()[i]*180/math.pi)    #Get the current joint angles in degrees
print("joint angles in degrees : ",joint_angles)

group.clear_pose_targets()   #Clear the target

moveit_commander.roscpp_shutdown()


# Move it cmdline interface : rosrun moveit_commander moveit_commander_cmdline.py