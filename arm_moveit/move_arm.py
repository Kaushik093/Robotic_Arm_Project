#! /usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler
import math
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node("move_group_python",anonymous=True)
group=moveit_commander.MoveGroupCommander("manipulator")


#Publish to /move_group/display_planned_path topic in Rviz to display trajectory on rviz
display_trajectory_publisher=rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=10)

# DISPLAY TRAJECTORY ON GAZEBO

print("Initial pose of end effector : ")
print(group.get_current_pose())    #Current end effector pose

joint_angles=[]
joint_angles.append(group.get_current_joint_values())  


pose_target= geometry_msgs.msg.Pose()

# Setting target pose for end effector

q = quaternion_from_euler(0,0,0) 

pose_target.orientation.w=q[3]
pose_target.orientation.x=q[2]
pose_target.orientation.y=q[1]
pose_target.orientation.z=q[0]

pose_target.position.x=-0.29
pose_target.position.y=0.0
pose_target.position.z=0.2  

group.set_pose_target(pose_target)
plan1=group.plan()       #Plan the path

group.go(wait=True)   #Execute the path on gazebo

# print("Final end effector pose :")
# print(group.get_current_pose())  
# joint_angles.append(group.get_current_joint_values())
# print(joint_angles)
# rospy.sleep(5)
# moveit_commander.roscpp_shutdown()
