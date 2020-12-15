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

        rospy.set_param('ekf_std_range', 10000) # from measurement
        
        self.M_Imu = np.zeros((3,6))
        self.M_Imu[:,3:6] = np.eye(3) 
        rospy.set_param('ekf_std_v', 1000000) # from measurement

        self.M_depth = np.matrix([0, 0, 1, 0, 0, 0])
        rospy.set_param('std_depth', 1)

        rospy.set_param('ekf_model_std_x', 1) # model
        rospy.set_param('ekf_model_std_y', 1)
        rospy.set_param('ekf_model_std_z', 1)
        rospy.set_param('ekf_model_std_v', 1000)  #  (for all 3 directions the same)

        # SUBSCRIBER:
        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.update_imu)
        rospy.Subscriber("ranges", RangeMeasurementArray, self.update_ranges)
        rospy.Subscriber("depth/state", Float64, self.update_depth)

        # PUBLISHER:
        self.pub_position = rospy.Publisher("localization/ekf/camera_position", Point, queue_size=1)
        self.pub_velocity = rospy.Publisher("localization/ekf/velocity", Point, queue_size=1)

    def update_ranges(self, msg):
        #self.state_vector, self.P = self.prediction_step(self.state_vector,self.P)
        R = np.eye(4)*rospy.get_param('ekf_std_range')
        H = self.get_jacobi_matrix(msg)
        z = np.zeros((len(msg.measurements), 1))
        # print('measurement vector')
        for measurement, i in zip(msg.measurements, range(len(msg.measurements))):
            z[i] = measurement.range # nach id sortiert?
        # print("z: ", z, np.size(z))
        # z = np.transpose(z)
        # print("z: ", z, np.size(z))
        y = z - self.mapping_state_to_range(msg)
        # print('y')
        # print(y,np.size(y))
        S = np.dot(np.dot(H, self.P), np.transpose(H)) + R[0:len(z), 0:len(z)]
        # print('S')
        # print(S,np.size(S))
        K = np.dot(np.dot(self.P, np.transpose(H)), np.linalg.inv(S))  # (6xvariable)
        # print('K')       
        # print(K,np.size(K))

        # Update state vector
        self.state_vector = self.state_vector + np.dot(K,y)
        # print('Updated_state vector')
        # print(self.state_vector, np.size(self.state_vector))
        self.P = np.dot((np.eye(6) - np.dot(K,H)), self.P)
        # print('P updated ')
        # print(self.P,np.size(self.P))
        self.publish_states()

    def mapping_state_to_range(self, msg):
        # Where are tag locations defined?
        ranges = np.zeros((len(msg.measurements), 1))
        for i, measurement in zip(range(len(msg.measurements)), msg.measurements):
            ranges[i] = np.linalg.norm(self.tags[measurement.id - 1] - self.state_vector[0:3])
        # print('Calculated ranges: ', ranges, np.size(ranges))
        return ranges

    def get_jacobi_matrix(self, msg):
        """
        Calculate the jacobian for the extended Kalman filter
        """
        jacobi_matrix = np.zeros((len(msg.measurements), 6))
        # print('self.state_vector[0:3] , in the get_jacobi matrix function')
        # print(self.state_vector[0:3])
        x = self.state_vector[0:3]
        for i, measurement in zip(range(len(msg.measurements)), msg.measurements):
            # print ('tags', measurement.id)
            tag_r = self.tags[measurement.id - 1]
            # print(tag_r)
            jacobi_matrix[i, 0:3] = np.transpose(x - tag_r) / np.linalg.norm(tag_r - x)
        # print('jacobian_matrix\n')
        # print(jacobi_matrix)
        # print('size of Jacobian',np.size(jacobi_matrix))
        return jacobi_matrix

    def update_depth(self, msg):
        R = np.matrix(rospy.get_param('std_depth'))
        z = msg.data + 0.1  # global formulieren
        y = z - np.dot(self.M_depth, self.state_vector)
        S = np.dot(np.dot(self.M_depth, self.P), np.transpose(self.M_depth)) + R
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

    def update_imu(self, msg):
        z = np.array([[-msg.twist.linear.y, msg.twist.linear.x, msg.twist.linear.z]]).T   # for IMU
        # print("z: " + str(z))
        y = z - np.dot(self.M_Imu, self.state_vector)
        # print("y: " + str(y))
        R =  np.eye(3)*rospy.get_param('ekf_std_v')
        # print("R: " + str(R))
        S = np.dot(np.dot(self.M_Imu, self.P), np.transpose(self.M_Imu)) + R
        # print("S: " + str(S))
        K = np.dot(np.dot(self.P, np.transpose(self.M_Imu)), np.linalg.inv(S))  # 6x3
        self.state_vector = self.state_vector + np.dot(K, y)
        # print("state_vector: " + str(self.state_vector))
        self.P = np.dot((np.eye(6) - np.dot(K, self.M_Imu)), self.P)
        # print("P: " + str(self.P))
        self.publish_states()

    def prediction_step(self):
        dt = 1 / self.prediction_rate
        F = np.eye(6)
        F[0:3,3:6] = np.eye(3)*dt
        # print('F' , F)
        # print("F: " + str(F))
        self.state_vector = np.dot(F, self.state_vector)
        # print('self.state_vector in prediction step ', x_sv)
        Q = np.diag([
            rospy.get_param('ekf_model_std_x'),
            rospy.get_param('ekf_model_std_y'),
            rospy.get_param('ekf_model_std_z'),
            rospy.get_param('ekf_model_std_v'),
            rospy.get_param('ekf_model_std_v'),
            rospy.get_param('ekf_model_std_v')])
        self.P = np.dot(np.dot(F, self.P), np.transpose(F)) + Q
        # print('P in prediction step ',P)



    def publish_states(self):
        position_msg = Point()
        # print("State vector befor publish: " + str(self.state_vector) + " size: " + str(np.size(self.state_vector)))
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
            self.prediction_step()
            rate.sleep()

def main():
    node = ExtendedKalmanFilter("ExtendedKalmanFilter")
    node.run()

if __name__ == "__main__":
   main()
