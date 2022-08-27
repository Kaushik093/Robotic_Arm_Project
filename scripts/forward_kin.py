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

    Transformation_matrix_pos=np.identity(4)
    Transformation_matrix_orient=np.identity(4)
    
    for i in range(6):
        

        temp_matrix=np.array([[cos(DH_matrix[i][0]), -cos(DH_matrix[i][3])*sin(DH_matrix[i][0]) , sin(DH_matrix[i][3])*sin(DH_matrix[i][0]) , DH_matrix[i][2]*cos(DH_matrix[i][0]) ],

                        [ sin(DH_matrix[i][0]), cos(DH_matrix[i][3])*cos(DH_matrix[i][0]) , -sin(DH_matrix[i][3])*cos(DH_matrix[i][0]) , DH_matrix[i][2]*sin(DH_matrix[i][0]) ],

                        [ 0  ,  sin(DH_matrix[i][3]) , cos(DH_matrix[i][3]) , DH_matrix[i][1] ],

                        [0 , 0 , 0 , 1]
                        
                        ])
        if(i>3): 
            Transformation_matrix_orient = MultiplyMatrix(Transformation_matrix_orient , temp_matrix)
        else:
            Transformation_matrix_pos = MultiplyMatrix( Transformation_matrix_pos , temp_matrix )
    
    Final_matrix=MultiplyMatrix(Transformation_matrix_pos , Transformation_matrix_orient)
    
    

    print(Final_matrix)

   
def MultiplyMatrix(A,B):
   
    result= [[0,0,0,0],
		[0,0,0,0],
		[0,0,0,0],
        [0,0,0,0]]

    result = np.dot(A,B)

    # print(result)
    return result


Calculate_Position_matrix()


# print(DH)
