#!/usr/bin/env python
import tf.transformations as tf
import numpy as np
import rospy

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

def convert_quaternion_to_array( q):
    return np.array([q.x,q.y,q.z,q.w])

def convert_point_to_array( point):
    return np.array([point.x, point.y, point.z])

def convert_array_to_quaternion( q):
    output = Quaternion()
    output.x = q[0]
    output.y = q[1]
    output.z = q[2]
    output.w = q[3]
    return output

def convert_array_to_point(p):
    output = Point()
    output.x = p[0]
    output.y = p[1]
    output.z = p[2]
    return output

def rotate_vector_by_quaternion( vec, quat):
    vecAsQuat = np.concatenate((vec, [0]))
    temp = tf.quaternion_multiply(quat, vecAsQuat)
    return np.array(tf.quaternion_multiply(temp, tf.quaternion_conjugate(quat))[0:3])

def convert_quaternion_to_globalsytem(q_rov_global, q_tag_relative):
    return np.array(tf.quaternion_multiply(q_rov_global, q_tag_relative))
