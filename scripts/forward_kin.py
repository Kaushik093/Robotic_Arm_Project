import numpy as np
from numpy import array
import math


sin=float(math.sin)
cos=float(math.cos)
pi=np.pi

print(type(sin))

# Joint angles 

t1=pi/2
t2=-pi/2
t3=pi/2
t4=0
t5=pi/2
t6=0

# DH parameter matrix [theta d a {alpha}] for Intelledex 660T

DH_matrix= np.array([
    [t1,373.4,0,pi/2],
    [t2,0,0,pi/2],
    [t3,0,304.8,0],
    [t4,0,304.8,0],
    [t5,0,0,pi/2],
    [t6,228.6,0,0]
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
        print(temp_matrix)
        print('\n')
        

        Transformation_matrix= MultiplyMatrix( Transformation_matrix , temp_matrix )
    
    # print(Transformation_matrix)

 
    
    
def MultiplyMatrix(A,B):
   
    result= [[0,0,0,0],
		[0,0,0,0],
		[0,0,0,0],
        [0,0,0,0]]

    result = np.dot(A,B)

    # print(result)
    return result


Calculate_Position_matrix()


# print(sin(DH_matrix[0][3]))


# print(DH)
