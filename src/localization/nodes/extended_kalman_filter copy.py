#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovariance 
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement
from std_msgs.msg import Float64

class ExtendedKalmanFilter():
    def __init__(self, name):
        rospy.init_node(name)

        rospy.set_param('prediction_rate',20.0)
        self.prediction_rate = rospy.get_param('prediction_rate')

       # rospy.set_param('tag1',[0.5, 3.35,-0.5])
        self.tag1 = np.array([[0.5, 3.35,-0.5]]).T

        self.tags = [None]*4
        for i in range(4):
            self.tags[i] = self.get_tag_coordinates(i+1)

        self.state_vector = np.zeros((6,1)) # 6x1 state vevtor with [x, y, z, v_x, v_y, v_z]
        self.P = np.eye(6)

        self.last_prediction = 0.0

        self.M_camera = np.eye(3, 6)
        rospy.set_param('std_tag1', 1000) # from measurement
        rospy.set_param('std_tag2', 1000)
        rospy.set_param('std_tag3', 1000)
        rospy.set_param('std_tag4', 1000)

        
        # self.M_Imu = np.zeros((3,6))
        # self.M_2[:,3:6] = np.eye(3) 
        # rospy.set_param('std_v_x', 100000) # from measurement
        # rospy.set_param('std_v_y', 100000)
        # rospy.set_param('std_v_z', 100000)
        self.M_depth = np.matrix([0, 0, 1, 0, 0, 0])

        rospy.set_param('std_depth', 1)

        rospy.set_param('model_std_x', 1) # model
        rospy.set_param('model_std_y', 1)
        rospy.set_param('model_std_z', 1)
        rospy.set_param('model_std_v_x', 1000)
        rospy.set_param('model_std_v_y', 1000)
        rospy.set_param('model_std_v_z', 1000)

        self.R_ranges = np.diag([rospy.get_param('std_tag1'), rospy.get_param('std_tag2'), rospy.get_param('std_tag3'), rospy.get_param('std_tag4')])
        self.R_depth = np.matrix(rospy.get_param('std_depth'))

        # SUBSCRIBER:
        # rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.update_imu)
        rospy.Subscriber("ranges", RangeMeasurementArray, self.update_ranges)
        rospy.Subscriber("depth/state", Float64, self.update_depth)

        # PUBLISHER:
        self.pub_position = rospy.Publisher("localization/ekf/camera_position", Point, queue_size=1)
        self.pub_velocity = rospy.Publisher("localization/ekf/velocity", Point, queue_size=1)

    def update_ranges(self, msg):
        self.state_vector, self.P = self.prediction_step(self.state_vector,self.P)
        H = self.get_jacobi_matrix()
        z = np.zeros((4,1))
        print('measurement vector')
        for measurement in msg.measurements:
            z[measurement.id-1] = measurement.range
        print("z: ",z,np.size(z))
        y = z - self.mapping_state_to_range()
        print('y')
        print(y,np.size(y))
        S = np.dot(np.dot(H, self.P), np.transpose(H)) + self.R_ranges # 4x4
        print('S')
        print(S,np.size(S))
        K = np.dot(np.dot(self.P, np.transpose(H)), np.linalg.inv(S))  # 6x4
        print('K')       
        print(K,np.size(K))

        # Update state vector
        self.state_vector = self.state_vector + np.dot(K,y)
        print('Updated_state vector')
        print(self.state_vector, np.size(self.state_vector))
        self.P = np.dot((np.eye(6) - np.dot(K,H)), self.P)
        print('P updated ')
        print(self.P,np.size(self.P))
        self.publish_states()

    def mapping_state_to_range(self):
        # Where are tag locations defined?
        ranges = np.zeros((4,1))
        for i in range(4):
            ranges[i] = np.linalg.norm(self.tags[i] - self.state_vector[0:3])
        print('Calculated ranges: ',ranges,np.size(ranges))
        return ranges

    def get_jacobi_matrix(self):
        """
        Calculate the jacobian for the extended Kalman filter
        """
        jacobi_matrix = np.zeros((4,6))
        print('self.state_vector[0:3] , in the get_jacobi matrix function')
        print(self.state_vector[0:3])
        for i in range(4):
            print ('tags',i)
            print(self.tags[i])

            jacobi_matrix[i,0:3] = np.transpose(self.state_vector[0:3] - self.tags[i]) / np.linalg.norm(self.tags[i] - self.state_vector[0:3])
        print('jacobian_matrix\n')
        print(jacobi_matrix)
        print('size of Jacobian',np.size(jacobi_matrix))
        return jacobi_matrix

    def update_depth(self, msg):
        z = msg.data
        y = z - np.dot(self.M_depth, self.state_vector)
        S = np.dot(np.dot(self.M_depth, self.P), np.transpose(self.M_depth)) + self.R_depth
        K = np.dot(np.dot(self.P, np.transpose(self.M_depth)), np.linalg.inv(S))  # 6x3
        self.state_vector = self.state_vector + np.dot(K,y)
        self.P = np.dot((np.eye(6) - np.dot(K, self.M_depth)), self.P)
        self.publish_states()

    def get_tag_coordinates(self, id):
        coordinates = [self.tag1, 
                       self.tag1 + np.array([[0.6, 0.0,  0.0]]).T,
                       self.tag1 + np.array([[0.0, 0.0, -0.4]]).T,
                       self.tag1 + np.array([[0.6, 0.0, -0.4]]).T]
        return coordinates[id-1]

    # def update_imu(self, msg):
    #     z = np.array([-msg.twist.linear.y, msg.twist.linear.state_vector, msg.twist.linear.z])   # for IMU
    #     # z = -np.array([msg.twist.twist.linear.state_vector, msg.twist.twist.linear.y, msg.twist.twist.linear.z]) # for ground truth
    #     # print("z: " + str(z))
    #     y = z - np.dot(self.M_2, self.state_vector)
    #     # print("y: " + str(y))
    #     R = np.diag([rospy.get_param('std_v_x'), rospy.get_param('std_v_y'), rospy.get_param('std_v_z')])
    #     # print("R: " + str(R))
    #     S = np.dot(np.dot(self.M_2, self.P), np.transpose(self.M_2)) + R
    #     # print("S: " + str(S))
    #     K = np.dot(np.dot(self.P, np.transpose(self.M_2)), np.linalg.inv(S))  # 6x3
    #     # if np.sum(K) > 1:
    #     #     print("S: " + str(S))
    #     #     print("K: " + str(K))
    #     self.state_vector = self.state_vector + np.dot(K, y)
    #     # print("state_vector: " + str(self.state_vector))
    #     self.P = np.dot((np.eye(6) - np.dot(K, self.M_2)), self.P)
    #     # print("P: " + str(self.P))
    #     self.publish_states()

    def prediction_step(self, x_sv, P):

        dt = 1 / self.prediction_rate
        F = np.eye(6)
        F[0:3,3:6] = np.eye(3)*dt
        # print('F' , F)
        # print("F: " + str(F))
        x_sv = np.dot(F, x_sv)
        print('self.state_vector in prediction step ', x_sv)
        Q = np.diag([
            rospy.get_param('model_std_x'),
            rospy.get_param('model_std_y'),
            rospy.get_param('model_std_z'),
            rospy.get_param('model_std_v_x'),
            rospy.get_param('model_std_v_y'),
            rospy.get_param('model_std_v_z')])
        P = np.dot(np.dot(F, P), np.transpose(F)) + Q
        print('P in prediction step ',P)
        return x_sv,P


    def publish_states(self):
        position_msg = Point()
        print("State vector befor publish: " + str(self.state_vector) + " size: " + str(np.size(self.state_vector)))
        position_msg.x = self.state_vector[0]
        position_msg.y = self.state_vector[1]
        position_msg.z = self.state_vector[2]
        self.pub_position.publish(position_msg)
        velocity_msg = Point()
        velocity_msg.x = self.state_vector[3]
        velocity_msg.y = self.state_vector[4]
        velocity_msg.z = self.state_vector[5]
        self.pub_velocity.publish(velocity_msg)

    def run(self):
        rate = rospy.Rate(self.prediction_rate)
        while not rospy.is_shutdown():
            # self.prediction_step()
            rate.sleep()

def main():
    node = ExtendedKalmanFilter("ExtendedKalmanFilter")
    node.run()

if __name__ == "__main__":
   main()
