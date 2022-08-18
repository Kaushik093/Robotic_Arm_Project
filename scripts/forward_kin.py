import numpy as np
from numpy import *
import math
# store dh param in a matrix 
# Define Tk k-1 matrix 

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

def calculate_matrix():
    
    for i in range(2):
        for j in range(3):
            Transform_matrix=np.array([cos(DH_matrix[i][i]),-cos(DH_matrix[i][i])*sin(DH_matrix[i][i]),sin(DH_matrix[i][i]*sin(DH_matrix))])
        
    

# print(DH_matrix[0][3])

# Transform=np.array([math.cos()])


# print(DH)
