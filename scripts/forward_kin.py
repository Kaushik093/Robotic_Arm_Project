from itertools import count
import numpy as np
import math

sin=math.sin
cos=math.cos
pi=np.pi

# Joint angles 

t1=pi/2
t2=-pi/2
t3=pi/2
t4=0
t5=pi/2
t6=0

# DH parameter [theta d a {alpha}] for Intelledex 660T

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

    euler_from_matrix(Final_matrix)

def euler_from_matrix(Final_matrix):

    orientation_vector=np.zeros(3)


    orientation_vector = np.array([[Final_matrix[0][0],Final_matrix[0][1],Final_matrix[0][2]],     # Extracting orientation vector from Final_matrix
                                    [Final_matrix[1][0],Final_matrix[1][1],Final_matrix[1][2]],
                                    [Final_matrix[2][0],Final_matrix[2][1],Final_matrix[2][2]],
                                    ],dtype=float)                                                      
    
    

    print(orientation_vector)


Calculate_matrix()

