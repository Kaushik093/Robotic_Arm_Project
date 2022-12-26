# PSO technique to optimise the path followed by the ur5 robot

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from pyswarm import pso
import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory


import rospy
from moveit_msgs.msg import DisplayTrajectory

# Define a callback function to handle incoming messages
def path_callback(msg):
    global path
    
    path = msg.trajectory[0]
    

# Subscribe to the /move_group/display_planned_path topic
rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, path_callback)

# Wait for the first message to be received on the subscribed topic
path = rospy.wait_for_message('/move_group/display_planned_path', DisplayTrajectory)

# Extract the poses and orientations from the planned path
poses = path.joint_trajectory.points
orientations = [pose.pose.orientation for pose in poses]

# Print the poses and orientations
print(poses)
print(orientations)




def forward_kinematics(joint_angles):
    # Define the DH parameters for the UR5e arm
    DH_params = [
        {'a': 0, 'alpha': np.pi/2, 'd': 0.089159, 'theta': joint_angles[0]},
        {'a': -0.425, 'alpha': 0, 'd': 0, 'theta': joint_angles[1] - np.pi/2},
        {'a': -0.39225, 'alpha': 0, 'd': 0, 'theta': joint_angles[2]},
        {'a': 0, 'alpha': np.pi/2, 'd': 0.10915, 'theta': joint_angles[3]},
        {'a': 0, 'alpha': -np.pi/2, 'd': 0.09465, 'theta': joint_angles[4]},
        {'a': 0, 'alpha': 0, 'd': 0.0823, 'theta': joint_angles[5]}
    ]

    # Initialize the transformation matrix
    T = np.eye(4)

    # Compute the forward kinematics using the DH convention
    for i, param in enumerate(DH_params):
        a = param['a']
        alpha = param['alpha']
        d = param['d']
        theta = param['theta']
        T_i = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        T = T @ T_i

    # Extract the position of the end effector from the transformation matrix
    x, y, z = T[:3, 3]
    return np.array([x, y, z])


def optimize_path(path):
    # Set the bounds for the PSO algorithm
    bounds = [(0, 2*np.pi)] * 6 * len(path)

    # Set the initial guess for the PSO algorithm
    x0 = initial_joint_angles * len(path)

    # Call the PSO algorithm to compute the optimal joint angles
    xopt, fopt = pso(objective_function, bounds, x0, maxiter=100, swarmsize=50)

    # Return the optimal joint angles
    print(xopt)

def objective_function(x):
    # Convert the joint angles to a joint state message
    joint_state = JointState()
    joint_state.position = x

    # Compute the forward kinematics for the arm
    end_effector_poses = [forward_kinematics(joint_state.position)]
    for i in range(1, len(path)):
        joint_state.position = x[i*6:(i+1)*6]
        end_effector_poses.append(forward_kinematics(joint_state.position))

    # Compute the total distance traveled by the end effector along the path
    distance = 0
    for i in range(1, len(path)):
        distance += np.linalg.norm(end_effector_poses[i] - end_effector_poses[i-1])
    return distance


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('PSO')

    # Set the initial joint angles
    global initial_joint_angles
    initial_joint_angles = [0, 0, 0, 0, 0, 0]

    # Create publishers for the joint angle topics
    global joint_angle_publishers

    optimize_path(path)
