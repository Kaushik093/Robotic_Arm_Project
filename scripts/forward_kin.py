import numpy as np
from numpy import *
import math

# Joint angles 
t1=math.pi/2
t2=0
t3=math.pi/2
t4=0
t5=math.pi

sin=math.sin
cos=math.cos
# DH parameter matrix [theta d a {alpha}] 
# parameters for Alpha 2 arm 5 DOF

DH_matrix= np.array([
    [t1,215,0,-math.pi/2],
    [t2,0,177.8,0],
    [t3,0,177.8,0],
    [t4,0,0,-math.pi/2],
    [t5,129.5,0,0]
    ])

# Transformation matrix for frame k-1 to k

# Wrist to base transformation matrix : To control position 

def Calculate_Position_matrix():

    Transformation_matrix=np.identity(4)
    
    for i in range(3):
            
        temp_matrix=np.array([[cos(DH_matrix[i][0]), -cos(DH_matrix[i][3])*sin(DH_matrix[i][0]) , sin(DH_matrix[i][3])*sin(DH_matrix[i][0]) , DH_matrix[i][2]*cos(DH_matrix[i][0]) ],

                        [ sin(DH_matrix[i][0]), cos(DH_matrix[i][3])*cos(DH_matrix[i][0]) , -sin(DH_matrix[i][3])*cos(DH_matrix[i][0]) , DH_matrix[i][2]*sin(DH_matrix[i][0]) ],

                        [ 0  ,  sin(DH_matrix[i][3]) , cos(DH_matrix[i][3]) , DH_matrix[i][1] ],

                        [0 , 0 , 0 , 1]
                        
                        ]) 

        Transformation_matrix= MultiplyMatrix( Transformation_matrix , temp_matrix )
    
    print(Transformation_matrix)
 
 
    
    
def MultiplyMatrix(A,B):
   
    result= [[0,0,0,0],
		[0,0,0,0],
		[0,0,0,0],
        [0,0,0,0]]

    result = np.dot(A,B)

    return result


Calculate_matrix()


# print(sin(DH_matrix[0][3]))


# print(DH)
