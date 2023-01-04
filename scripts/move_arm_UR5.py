#! /usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import numpy as np


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

        self.pose_target.position.x =   -0.4                 
        self.pose_target.position.y = 0.3   
        self.pose_target.position.z = -0.48778

        self.group.set_pose_target(self.pose_target)

        self.group.set_start_state_to_current_state()
        
        self.group.set_planner_id("RRTkConfigDefault")

        plan = self.group.plan()
        
        if plan[0]==True : 
            for pose in plan[1].joint_trajectory.points:
                position = pose.positions
                self.path.append(position) 
                self.group.go(wait=True)
        else :
            print("No path found")
            self.group.stop()

        # Calculate total distance travelled by the arm
        total_distance = 0
        for i in range(len(self.path)-1):
            total_distance += np.linalg.norm(np.array(self.path[i+1]) - np.array(self.path[i]))
        print("Total distance travelled: {}".format(total_distance))

            
        
        self.group.clear_pose_targets()
    def get_path(self):
        return self.path


# moveit_commander.roscpp_shutdown()
