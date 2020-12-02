#!/usr/bin/env python
import rospy
# from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from depth_controller.msg import Orientation
#from depth_controller/msg/Orientation.msg import Orientation
#from bluerov_sim.msg import ActuatorCommands

class PoseReader():
    def __init__(self, name):
        rospy.init_node(name)

        self.euler_pub = rospy.Publisher("orientation/euler", Orientation, queue_size=1)

        rospy.Subscriber("mavros/local_position/pose", PoseStamped,
                                    self.pose_callback)

    def pose_callback(self, msg):
        euler = Orientation()
        euler.header.stamp = rospy.Time.now()
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (euler.roll, euler.pitch, euler.yaw) = euler_from_quaternion(orientation_list)
        self.euler_pub.publish(euler)


def main():
    node = PoseReader("PoseReader")
    rospy.spin()

if __name__ == "__main__":
   main()
