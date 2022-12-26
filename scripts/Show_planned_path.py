# Import the moveit_commander library
import moveit_commander

path=[]
# Initialize the move group commander for the robot's arm

arm = moveit_commander.MoveGroupCommander("manipulator")

# Plan a path for the robot to follow
plan = arm.plan()

print("Printing the planned path...")
print(plan)
# Iterate through the poses in the planned path
for pose in plan[1].joint_trajectory.points:
    # Extract the position and orientation of the pose
    position = pose.positions
    

    # Print the position and orientation
    # print("Position:", position)
    path.append(position)
# Print the path
print(len(path))