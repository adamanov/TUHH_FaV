#!/usr/bin/env python
import rospy
import numpy as np
import math as m
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement
from depth_controller.msg import Orientation
from tf.transformations import euler_matrix
from scipy.optimize import lsq_linear


class PositionEstimator():
    def __init__(self, name):
        rospy.init_node(name)

        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.range_array = []
        self.depth = 0.0
        self.last_depth_time = 0.0
        self.depth_timeout = 2 * 10**9
        self.range_sensor_link = np.array([-0.2, 0.0, 0.1])

        self.euler = Orientation()

        # Adjust for the experiment!
        self.tag1_coor = np.array([0.72, 3.35, -0.28])

        self.lb = np.array([0.0, 0.0, -1.6])
        self.ub = np.array([2.0, 3.35, 0.2])
        self.lb_trans = np.array(
            [sum(np.square(self.lb)), self.lb[0]*2, self.lb[1]*2, self.lb[2]*2])
        self.ub_trans = np.array(
            [sum(np.square(self.ub)), self.ub[0]*2, self.ub[1]*2, self.ub[2]*2])
        self.lb_trans2 = np.array(
            [sum(np.square(self.lb[0:1])), self.lb[0]*2, self.lb[1]*2])
        self.ub_trans2 = np.array(
            [sum(np.square(self.ub[0:1])), self.ub[0]*2, self.ub[1]*2])

        rospy.Subscriber("ranges/average",
                         RangeMeasurementArray, self.range_callback)

        rospy.Subscriber("depth/state", Float64, self.depth_callback)

        rospy.Subscriber("orientation/euler", Orientation,
                         self.orient_callback,
                         queue_size=1)

        self.pub_position = rospy.Publisher(
            "localization/position_estimate", Point, queue_size=1)

    def range_callback(self, msg):
        self.range_array = msg.measurements

    def orient_callback(self, msg):
        self.euler = msg
        # print("roll: " + str(msg.roll*360/(2*m.pi)) + " pitch: " + str(msg.pitch*360/(2*m.pi)) + " yaw: " + str(msg.yaw*360/(2*m.pi)) )

    def depth_callback(self, msg):
        self.depth = msg.data
        self.last_depth_time = rospy.Time.now().nsecs

    def get_tag_coordinates(self, id):
        coordinates = [self.tag1_coor,
                       self.tag1_coor + np.array([0.6, 0.0, 0.0]),
                       self.tag1_coor + np.array([0.0, 0.0, -0.4]),
                       self.tag1_coor + np.array([0.6, 0.0, -0.4])]
        return coordinates[id-1]

    def is_current_depth(self):
        return (rospy.Time.now().nsecs - self.last_depth_time) < self.depth_timeout

    def estimate_current_position(self):
        num_tag_measurements = len(self.range_array)
        if True:  # self.is_current_depth():
            if num_tag_measurements == 0:
                pass
               # rospy.loginfo("I aint see no AruCo-Marker!")
            elif num_tag_measurements >= 2:
               #rospy.loginfo("I see at least one AruCo-Marker!")
                A = np.zeros((num_tag_measurements, 3))
                b = np.zeros(num_tag_measurements)
                for tag_range, i in zip(self.range_array, range(num_tag_measurements)):
                    tag_x, tag_y, tag_z = self.get_tag_coordinates(
                        tag_range.id)
                    A[i, :] = [1.0, -tag_x, -tag_y]
                    b[i] = tag_range.range**2 - tag_x**2 - tag_y**2 - \
                        (self.depth + self.range_sensor_link[2] -
                         tag_z)**2  # + self.range_sensor_link[2]
                # print("A: " + str(A))
                # print("b: " + str(b))
                # temp_states = np.transpose(np.array(np.linalg.lstsq(A, b)[0]))[0]
                temp_states = lsq_linear(A, np.array(b), bounds=(
                    self.lb_trans2, self.ub_trans2)).x
                self.current_pos = np.append(self.transform_into_coor(
                    temp_states), self.depth + self.range_sensor_link[2])
                # print("estimated pos: " + str(self.current_pos))
                msg = Point()
                temp = np.array(
                    self.current_pos) - np.array(self.rotate_from_relK_to_initK(self.range_sensor_link))
                # print("position: " + str(temp))
                msg.x, msg.y, msg.z = temp[0]
                self.pub_position.publish(msg)
        else:
            rospy.logwarn("No depth measurement received for " +
                          str(rospy.Time.now().nsec - self.last_depth_time) + " seconds")

    def transform_into_coor(self, temp_states):
        return np.array(temp_states[1:]/2)

    def rotate_from_relK_to_initK(self, vec):
        rot_matrix = self.rotation_matrix_from_euler(self.euler)
        # print(rot_matrix)
        rot_vec = np.dot(vec, rot_matrix)
        # print("rot_vec: " + str(rot_vec))
        return rot_vec[0]

    def rotation_matrix_from_euler(self, euler):
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

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.estimate_current_position()
            rate.sleep()


def main():
    node = PositionEstimator("PositionEstimator")
    node.run()


if __name__ == "__main__":
    main()

    # def estimate_current_position(self):
    #     num_tag_measurements = len(self.range_array)
    #     if True: #self.is_current_depth():
    #          if num_tag_measurements == 0:
    #             rospy.loginfo("I aint see no AruCo-Marker!")
    #          elif num_tag_measurements >= 2 :
    #             #rospy.loginfo("I see at least one AruCo-Marker!")
    #             #s0 = np.array([0, 0, 0])
    #             s0 = np.array(self.current_pos)                     # hier z hinzuaddieren?
    #             # print("s0: " + str(s0))
    #             A = np.zeros((num_tag_measurements + 1, 3))
    #             b = np.zeros((num_tag_measurements + 1))
    #             for tag_range, i in zip(self.range_array, range(num_tag_measurements)):
    #                 dif = s0 - np.array(self.get_tag_coordinates(tag_range.id))   # column vector
    #                 grad = dif/np.linalg.norm(dif)                          # column vector
    #                 error = np.linalg.norm(dif) - tag_range.range                 # scalar
    #                 A[i,:] = grad
    #                 b[i] = -error + np.dot(grad, s0)
    #             A[-1, :] = np.array([0, 0, 1])
    #             b[-1] = self.depth + self.range_sensor_link[2]
    #             # print("A: " + str(A))
    #             # print("b: " + str(b))
    #             # self.current_pos = np.transpose(np.array(np.linalg.lstsq(A, b)[0]))[0]
    #             self.current_pos = lsq_linear(A, np.array(b), bounds=(self.lb, self.ub)).x
    #             # print("estimated pos: " + str(self.current_pos))
    #             msg = Point()
    #             msg.x, msg.y, msg.z = (np.array(self.current_pos) - np.array(self.rotate_from_relK_to_initK(self.range_sensor_link)))[0]
    #             self.pub_position.publish(msg)
    #     else:
    #          rospy.logwarn("No depth measurement received for " + str(rospy.Time.now().nsec - self.last_depth_time) + " seconds")

    # def estimate_current_position(self):
    #     num_tag_measurements = len(self.range_array)
    #     if True: #self.is_current_depth():
    #          if num_tag_measurements == 0:
    #             rospy.loginfo("I aint see no AruCo-Marker!")
    #          elif num_tag_measurements >= 1 :
    #             #rospy.loginfo("I see at least one AruCo-Marker!")
    #             A = np.zeros((num_tag_measurements+1, 4))
    #             b = np.zeros(num_tag_measurements+1)
    #             for tag_range, i in zip(self.range_array, range(num_tag_measurements)):
    #                 tag_x, tag_y, tag_z = self.get_tag_coordinates(tag_range.id)
    #                 A[i,:] = [1.0, -tag_x, -tag_y, -tag_z]
    #                 b[i] = tag_range.range**2 - tag_x**2 - tag_y**2 - tag_z**2
    #             A[-1, :] = np.array([0, 0, 0, 1])
    #             b[-1] = self.depth*2
    #             # print("A: " + str(A))
    #             # print("b: " + str(b))
    #             # temp_states = np.transpose(np.array(np.linalg.lstsq(A, b)[0]))[0]
    #             temp_states = lsq_linear(A, np.array(b), bounds=(self.lb_trans, self.ub_trans)).x
    #             self.current_pos = self.transform_into_coor(temp_states)
    #             # print("estimated pos: " + str(self.current_pos))
    #             msg = Point()
    #             msg.x, msg.y, msg.z = self.current_pos # - self.rotate_from_relK_to_initK(self.range_sensor_link)
    #             self.pub_position.publish(msg)
    #     else:
    #          rospy.logwarn("No depth measurement received for " + str(rospy.Time.now().nsec - self.last_depth_time) + " seconds")

    # def two_maker_and_depth(self):
    #     [x0, y0, z0] = self.get_tag_coordinates(self.range_array[0].id)
    #     [x1, y1, z1] = self.get_tag_coordinates(self.range_array[1].id)
    #     d0 = self.range_array[0].range
    #     d1 = self.range_array[1].range
    #     z = self.depth
    #     x = (x0**2+z0**2-2*z0*z-x1**2-z1**2+2*z1*z-d0**2+d1**2) / (2*(x0-x1))
    #     y = 0.5 * (2*y0 + 2*np.sqrt(2*x0*x-x**2-x0**2-z0**2+2*z0*z-z**2+d0**2))
    #     return [x, y, z]

    # def kalman_filter_update(x, z, A, H, K, P, R, Q):
    #     x_prior = np.dot(A,)

    # def estimate_current_position(self):
    #     num_tag_measurements = len(self.range_array)
    #     if True: #self.is_current_depth():
    #         if num_tag_measurements == 0:
    #             rospy.loginfo("I aint see no AruCo-Marker!")
    #         elif num_tag_measurements == 1 :
    #             rospy.loginfo("I can see ONE AruCo-Marker!")
    #         elif num_tag_measurements >= 2 :
    #             rospy.loginfo("I can see TWO OR MORE AruCo-Marker!")
    #             self.current_pos = self.two_maker_and_depth()
    #             msg = Point()
    #             msg.x, msg.y, msg.z = self.current_pos
    #             self.pub_position.publish(msg)
    #     else:
    #         rospy.logwarn("No depth measurement received for " + str(rospy.Time.now().nsec - self.last_depth_time) + " seconds")

    # def estimate_current_position(self):
    #     num_tag_measurements = len(self.range_array)
    #     if True: #self.is_current_depth():
    #          if num_tag_measurements == 0:
    #             rospy.loginfo("I aint see no AruCo-Marker!")
    #          elif num_tag_measurements >= 1 :
    #             #rospy.loginfo("I see at least one AruCo-Marker!")
    #             #s0 = np.array([0, 0, 0])
    #             s0 = np.array(self.current_pos)
    #             print("s0: " + str(s0))
    #             A = np.zeros((num_tag_measurements + 1, 3))
    #             b = np.zeros((num_tag_measurements + 1,1))
    #             for tag_range, i in zip(self.range_array, range(num_tag_measurements)):
    #                 dif = s0 - np.array(self.get_tag_coordinates(tag_range.id))   # column vector
    #                 grad = dif/np.linalg.norm(dif)                          # column vector
    #                 error = np.linalg.norm(dif) - tag_range.range                 # scalar
    #                 A[i,:] = grad
    #                 b[i] = -error + np.dot(grad, s0)
    #             A[-1, :] = np.array([0, 0, 1])
    #             b[-1] = self.depth
    #             print("A: " + str(A))
    #             print("b: " + str(b))
    #             self.current_pos = np.transpose(np.array(np.linalg.lstsq(A, b)[0]))[0]
    #             print("estimated pos: " + str(self.current_pos))
    #             msg = Point()
    #             msg.x, msg.y, msg.z = self.current_pos # - self.rotate_from_relK_to_initK(self.range_sensor_link)
    #             self.pub_position.publish(msg)
    #     else:
    #          rospy.logwarn("No depth measurement received for " + str(rospy.Time.now().nsec - self.last_depth_time) + " seconds")
