#! /usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler


class get_path:
    def __init__(self):
        self.path = []
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.pose_target = geometry_msgs.msg.Pose()

    def move(self):
        q = quaternion_from_euler(0, 0, 0)
        self.pose_target.orientation.x = q[0]
        self.pose_target.orientation.y = q[1]
        self.pose_target.orientation.z = q[2]
        self.pose_target.orientation.w = q[3]
        self.pose_target.position.x = -0.7
        self.pose_target.position.y = -0.2
        self.pose_target.position.z = 0.3
        self.group.set_pose_target(self.pose_target)
        self.group.set_planner_id("RRTstarkConfigDefault")
        plan = self.group.plan()
        self.group.go(wait=True)
        for pose in plan[1].joint_trajectory.points:
            position = pose.positions
            self.path.append(position)    # path is a list of joint angles at each point

        self.group.stop()
        self.group.clear_pose_targets()
    def get_path(self):
        return self.path

rospy.init_node("move_group_python", anonymous=True)

# moveit_commander.roscpp_shutdown()
