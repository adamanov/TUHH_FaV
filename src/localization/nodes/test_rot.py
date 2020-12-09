import numpy as np
from depth_controller.msg import Orientation
import math as m

euler = Orientation()
euler.roll, euler.pitch, euler.yaw = 0.0*2*m.pi/360, 0.0*2*m.pi/360, - 90.0*2*m.pi/360

# vec_cam = np.array([1.0, 1.0, 0.0])
vec_rob = np.array([1.0, 0.0, 0.0])
vec = np.array([1.0, 0.0, 0.0])


def rotate_from_relK_to_initK(vec):
        rot_matrix = rotation_matrix_from_euler(euler)
        print(rot_matrix)
        rot_vec = np.dot(vec, rot_matrix)
        print("rot_vec: " + str(rot_vec)) 
        return rot_vec[0]
    
def rotation_matrix_from_euler(euler):
        roll, pitch, yaw = euler.roll, euler.pitch, euler.yaw
        yawMatrix = np.matrix([
        [m.cos(yaw), -m.sin(yaw), 0],
        [m.sin(yaw), m.cos(yaw), 0],
        [0, 0, 1]
        ])
        pitchMatrix = np.matrix([
        [m.cos(pitch), 0, m.sin(pitch)],
        [0, 1, 0],
        [-m.sin(pitch), 0, m.cos(pitch)]
        ])
        rollMatrix = np.matrix([
        [1, 0, 0],
        [0, m.cos(roll), -m.sin(roll)],
        [0, m.sin(roll), m.cos(roll)]
        ])
        return np.dot(np.dot(yawMatrix, pitchMatrix), rollMatrix) 

print(np.array(vec_rob) + np.array(rotate_from_relK_to_initK(vec)))

rot_vec = rotate_from_relK_to_initK(vec)
