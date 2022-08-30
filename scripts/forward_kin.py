# Python code for computing forward kinematics for any robot with any number of DOF


import numpy as np
import math

sin=math.sin
cos=math.cos
pi=np.pi

# Joint angles 

t1=pi/2
t2=-pi/2
t3=pi/2
t4=pi
t5=pi/2
t6=0

# DH parameter [theta d a {alpha}] for Intelledex 660T

# Change the DH parameters to get the forward kinematics for any robot

theta=[t1,t2,t3,t4,t5,t6]
d = [373.4,0,0,0,0,228.4]
a=[0,0,304.8,304.8,0,0]
alpha=[pi/2,pi/2,0,0,pi/2,0]

# Transformation matrix for frame k-1 to k

# Wrist to base transformation matrix : To control position 

def Calculate_matrix():

    Transformation_matrix_pos=np.identity(4)
    Transformation_matrix_orient=np.identity(4)
    
    for i in range(len(theta)):
        

        temp_matrix=np.array([[cos(theta[i]), -cos(alpha[i])*sin(theta[i]) , sin(alpha[i])*sin(theta[i]) , a[i]*cos(theta[i]) ],

                        [ sin(theta[i]), cos(alpha[i])*cos(theta[i]) , -sin(alpha[i])*cos(theta[i]) , a[i]*sin(theta[i]) ],

                        [ 0  ,  sin(alpha[i]) , cos(alpha[i]) , d[i] ],

                        [0 , 0 , 0 , 1]
                        
                        ])
                          
        if(i>3): 
            Transformation_matrix_orient = np.dot(Transformation_matrix_orient , temp_matrix)  #Computing orientation matrix 
        else:
            Transformation_matrix_pos =  np.dot( Transformation_matrix_pos , temp_matrix )
    
    Final_matrix=np.dot(Transformation_matrix_pos , Transformation_matrix_orient)
    

    position_vector = [Final_matrix[0][3] , Final_matrix[1][3] , Final_matrix[2][3]]

    print("Position: ",position_vector,"\n")

    euler_from_matrix(Final_matrix)


def euler_from_matrix(Final_matrix):

    orientation_vector = np.array([[Final_matrix[0][0],Final_matrix[0][1],Final_matrix[0][2]],     # Extracting orientation vector from Final_matrix
                                    [Final_matrix[1][0],Final_matrix[1][1],Final_matrix[1][2]],
                                    [Final_matrix[2][0],Final_matrix[2][1],Final_matrix[2][2]],
                                    ],dtype=float)  

    # Extracting Euler angles from orientation vector

    if(orientation_vector[2][0]!=1):
        pitch1=-math.asin(orientation_vector[2][0]) 
        pitch2= pi - pitch1

        roll1 = math.atan2(orientation_vector[2][1]/math.cos(pitch1),orientation_vector[2][2]/math.cos(pitch1))
        roll2 = math.atan2(orientation_vector[2][1]/math.cos(pitch2),orientation_vector[2][2]/math.cos(pitch2))

        yaw1 = math.atan2(orientation_vector[1][0]/math.cos(pitch1),orientation_vector[0][0]/math.cos(pitch1))
        yaw2 = math.atan2(orientation_vector[1][0]/math.cos(pitch2),orientation_vector[0][0]/math.cos(pitch2))

        print("Two solutions for orientation of end effector : \n")
        print("Roll: ",roll1*180/pi," Pitch: ",pitch1*180/pi," Yaw: ",yaw1*180/pi,"\n")
        print("Roll: ",roll2*180/pi," Pitch: ",pitch2*180/pi," Yaw: ",yaw2*180/pi)
    else :
        yaw=0  # can be set to any value
        if(orientation_vector[2][0]==-1):
            pitch = pi/2
            roll = yaw + math.atan2(orientation_vector[0][1],orientation_vector[0][2])
        else:
            pitch = -pi/2
            roll = -yaw + math.atan2(-orientation_vector[0][1],-orientation_vector[0][2])

        print("Orientation of end effector : \n")
        print("Roll: ",roll*180/pi," Pitch: ",pitch*180/pi," Yaw: ",yaw*180/pi)
         
    

Calculate_matrix()

