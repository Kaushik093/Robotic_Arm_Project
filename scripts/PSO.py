import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from pyswarm import pso
import numpy as np
from move_arm_UR5 import get_path

count =0

def forward_kinematics(joint_angles):
    # print("Joint angles: ",joint_angles)
    # Define the DH parameters for the arm
    # Each row represents a link and contains the following parameters:
    #  [alpha, a, d, theta]
    dh_params = np.array([[0, 0, 0, joint_angles[0]],
                         [np.pi/2, 0, 0, joint_angles[1]],
                         [0, 0.425, 0, joint_angles[2]],
                         [0, 0.39225, 0, joint_angles[3]],
                         [np.pi/2, 0, 0, joint_angles[4]],
                         [-np.pi/2, 0, 0, joint_angles[5]]])

    # Initialize the transformation matrix for the base frame
    T = np.identity(4)

    # Compute the transformation matrices for each link
    for i in range(dh_params.shape[0]):
        alpha, a, d, theta = dh_params[i]
        T_link = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                           [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                           [0, np.sin(alpha), np.cos(alpha), d],
                           [0, 0, 0, 1]])
        T = T @ T_link

    # Return the position and orientation of the end effector frame
    end_effector_pose = T[:3, 3]
    end_effector_orientation = T[:3, :3]
    return end_effector_pose, end_effector_orientation


def objective_function(x):

    global count
    
    # Convert the joint angles to a joint state message
    joint_state = JointState()
    joint_state.position = x
    
    # print(len(x))

    # Compute the forward kinematics for the arm
    end_effector_poses = [forward_kinematics(joint_state.position)]
    for i in range(1, len(path)):
        joint_state.position = x[i*6:(i+1)*6]
        end_effector_poses.append(forward_kinematics(joint_state.position))

    
    # Compute the total distance traveled by the end effector along the path
    distance = 0
    for i in range(1, len(path)):
        # print("Pose 1: {}".format(end_effector_poses[i][0]))
        # print("Pose 2: {}".format(end_effector_poses[i-1][0]))
        distance += np.linalg.norm(end_effector_poses[i][0] - end_effector_poses[i-1][0])
    
    print("Distance: {}".format(distance))
   

    return distance


def generate_trajectory(path):
    # Set the bounds for the PSO algorithm
    lower_bounds = [(0)] * 6 * len(path)
    upper_bounds = [(2*np.pi)] * 6 * len(path)

    # Set the initial guess for the PSO algorithm
    x0 = initial_joint_angles * len(path)

    # Call the PSO algorithm to compute the optimal joint angles
    xopt, fopt = pso(objective_function, lower_bounds, upper_bounds,  maxiter=100, swarmsize=50)   #fopt is the minimum distance 

    print("Optimal joint angles: {}".format(xopt))
    return xopt

robot = get_path()  
robot.move()
path = robot.get_path()   #Initial path to be optimized


initial_joint_angles = [0, 0, 0, 0, 0, 0]   #Initial joint angles

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

rospy.init_node("PSO")

trajectory = generate_trajectory(path)


# pub = rospy.Publisher("joint_states", JointState, queue_size=10)
# pub.publish(trajectory)
