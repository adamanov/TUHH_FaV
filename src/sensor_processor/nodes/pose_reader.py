#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from sensor_processor.msg import Orientation
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseReader():
    def __init__(self, name):
        rospy.init_node(name)

        # PUBLISHER:
        self.euler_pub = rospy.Publisher("orientation/euler", Orientation, queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw/state", Float64, queue_size=1)

        # SUBSCRIBER:
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_callback)
        #rospy.Subscriber("mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, self.pose_callback)

    def pose_callback(self, msg):
        euler = Orientation()
        euler.header.stamp = rospy.Time.now() # aendern zum timestamp vom msg
        orientation_q = msg.pose.orientation
        # orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (euler.roll, euler.pitch, euler.yaw) = euler_from_quaternion(orientation_list)
        self.euler_pub.publish(euler)
        msg_yaw = Float64()
        msg_yaw.data = euler.yaw
        self.yaw_pub.publish(msg_yaw)


def main():
    node = PoseReader("PoseReader")
    rospy.spin()

if __name__ == "__main__":
   main()
