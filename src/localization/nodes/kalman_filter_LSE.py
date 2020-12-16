#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovariance


class KalmanFilter():
    def __init__(self, name):
        rospy.init_node(name)

        self.x = np.zeros(6)  # 6x1 state vevtor with [x, y, z, v_x, v_y, v_z]
        self.P = np.eye(6)

        rospy.set_param('prediction_rate', 20.0)
        self.prediction_rate = rospy.get_param('prediction_rate')

        self.M_1 = np.eye(3, 6)
        rospy.set_param('lskf_std_x', 300000)  # from measurement
        rospy.set_param('lskf_std_y', 5000)
        rospy.set_param('lskf_std_z', 0.01)

        self.M_2 = np.zeros((3, 6))
        self.M_2[:, 3:6] = np.eye(3)
        rospy.set_param('lskf_std_v', 100000)  # from measurement, (for all 3 directions the same)

        rospy.set_param('lskf_model_std_x', 1)  # model
        rospy.set_param('lskf_model_std_y', 1)
        rospy.set_param('lskf_model_std_z', 1)
        rospy.set_param('lskf_model_std_v', 1000)  #  (for all 3 directions the same)

        # SUBSCRIBER:
        rospy.Subscriber("localization/least_squares/camera_position", Point, self.position_callback)
        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.imu_callback)
        # rospy.Subscriber("/mavros/local_position/velocity_body_cov", TwistWithCovariance, self.imu_callback)

        # PUBLISHER:
        self.pub_position = rospy.Publisher("localization/least_squares_and_kf/camera_position", Point, queue_size=1)
        self.pub_velocity = rospy.Publisher("localization/least_squares_and_kf/velocity", Point, queue_size=1)

    # Input ranges -> predict ranges using vetors, EKF?
    def position_callback(self, msg):
        z = np.array([msg.x, msg.y, msg.z])
        y = z - np.dot(self.M_1, self.x)
        R = np.diag([rospy.get_param('lskf_std_x'), rospy.get_param(
            'lskf_std_y'), rospy.get_param('lskf_std_z')])
        S = np.dot(np.dot(self.M_1, self.P), np.transpose(self.M_1)) + R
        K = np.dot(np.dot(self.P, np.transpose(self.M_1)),
                   np.linalg.inv(S))  # 6x3
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(6) - np.dot(K, self.M_1)), self.P)
        self.publish_states()

    def imu_callback(self, msg):
        z = np.array([-msg.twist.linear.y, msg.twist.linear.x,
                      msg.twist.linear.z])   # for IMU
        # print("z: " + str(z))
        y = z - np.dot(self.M_2, self.x)
        # print("y: " + str(y))
        R =  np.eye(3)*rospy.get_param('lskf_std_v')
        # print("R: " + str(R))
        S = np.dot(np.dot(self.M_2, self.P), np.transpose(self.M_2)) + R
        # print("S: " + str(S))
        K = np.dot(np.dot(self.P, np.transpose(self.M_2)),
                   np.linalg.inv(S))  # 6x3
        # print("K: " + str(K))
        self.x = self.x + np.dot(K, y)
        # print("x: " + str(self.x))
        self.P = np.dot((np.eye(6) - np.dot(K, self.M_2)), self.P)
        # print("P: " + str(self.P))
        self.publish_states()

    def prediction_step(self):
        dt = 1 / self.prediction_rate
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3)*dt
        # print("F: " + str(F))
        self.x = np.dot(F, self.x)
        Q = np.diag([
            rospy.get_param('lskf_model_std_x'),
            rospy.get_param('lskf_model_std_y'),
            rospy.get_param('lskf_model_std_z'),
            rospy.get_param('lskf_model_std_v'),
            rospy.get_param('lskf_model_std_v'),
            rospy.get_param('lskf_model_std_v')])
        self.P = np.dot(np.dot(F, self.P), np.transpose(F)) + Q

    def publish_states(self):
        msg = Point()
        msg.x, msg.y, msg.z = self.x[0:3]
        self.pub_position.publish(msg)
        msg2 = Point()
        msg2.x, msg2.y, msg2.z = self.x[3:6]
        self.pub_velocity.publish(msg2)

    def run(self):
        rate = rospy.Rate(self.prediction_rate)
        while not rospy.is_shutdown():
            self.prediction_step()
            rate.sleep()


def main():
    node = KalmanFilter("KalmanFilter")
    node.run()


if __name__ == "__main__":
    main()
