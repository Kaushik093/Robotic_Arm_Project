#!/usr/bin/env python

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import moveit_commander
import math


def main():

    group=moveit_commander.MoveGroupCommander("manipulator")
    rospy.init_node('send_joint_angle')
    
    pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    group.set_max_velocity_scaling_factor(0.2)
    print("initial joint angles : ",group.get_current_joint_values())
    # Joint names for arm
    traj.joint_names = ['joint1','joint2','joint3','joint4','joint5']
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()
        pts.positions = [-math.pi/2,-math.pi/3,0,0,0]
        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)
    print("joint angles : ",group.get_current_joint_values())


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")