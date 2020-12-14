#!/usr/bin/env python
import rospy
import numpy as np
import time
import math as m

from bluerov_sim.msg import ActuatorCommands
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float64
from depth_controller.msg import Orientation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped


class Controller():
    def __init__(self, name):
        rospy.init_node(name)

        self.rate = 20

        # self.arm_vehicle()

        # calculated actuator commands for mixer
        self.thrust = 0.0
        self.lateral_thrust = 0.0
        self.vertical_thrust = 0.0
        self.yaw_rate = 0.0
        self.pitch_rate = 0.0
        self.roll_rate = 0.0

        # Controller
        self.control_effort_vertical_thrust = 0.0
        self.control_effort_thrust = 0.0
        self.control_effort_lateral_thrust = 0.0
        self.control_effort_yaw = 0.0
        self.control_effort_pitch = 0.0
        self.control_effort_roll = 0.0

        # PUBLISHER:
        self.actuator_pub = rospy.Publisher("mixer/actuator_commands", ActuatorCommands, queue_size=1)

        # SUBSCRIBER:
        rospy.Subscriber("thrust/control_effort", Float64, self.control_callback_thrust, queue_size=1)
        rospy.Subscriber("vertical_thrust/control_effort", Float64, self.control_callback_vertical_thrust, queue_size=1)
        rospy.Subscriber("lateral_thrust/control_effort", Float64, self.control_callback_lateral_thrust, queue_size=1)
        rospy.Subscriber("yaw/control_effort", Float64, self.control_callback_yaw, queue_size=1)
        # rospy.Subscriber("pitch/control_effort", Float64, self.control_callback_pitch, queue_size=1)
        # rospy.Subscriber("roll/control_effort", Float64, self.control_callback_roll, queue_size=1)

    """ def arm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        while not arm(True).success:
            rospy.logwarn_throttle(1, "Could not arm vehicle. Keep trying.")
        rospy.loginfo("Armed successfully.")

    def disarm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        arm(False) """

    # ------------------------control_callback_vertical_thrust-------------------------------------------------------
    def control_callback_vertical_thrust(self, msg):
        self.control_effort_vertical_thrust = msg.data
        self.set_vertical_thrust(self.control_effort_vertical_thrust)
        # safezone_upper = rospy.get_param('safezone_upper')
        # safezone_lower = rospy.get_param('safezone_lower')
        # if (self.vertical_thrust_setpoint < safezone_upper and self.vertical_thrust_setpoint > safezone_lower):
        #     self.set_vertical_thrust(self.control_effort_vertical_thrust)
        #     # rospy.loginfo(" Congratulations You passed ALL CHECKS in depth!!!")
        # elif self.vertical_thrust_setpoint > safezone_upper:
        #     self.set_vertical_thrust(min(self.control_effort_vertical_thrust, 0))
        #     rospy.loginfo("over safezone_upper!!   (No positive control_effort allowed)")
        # elif self.vertical_thrust_setpoint < safezone_lower:
        #     self.set_vertical_thrust(max(self.control_effort_vertical_thrust, 0))
        #     rospy.loginfo("under safezone_lower!!   (No negative control_effort allowed)")
        # else:
        #     self.set_vertical_thrust(0.0)

    def control_callback_thrust(self, msg):
        self.control_effort_thrust = msg.data
        self.set_thrust(self.control_effort_thrust)

    def control_callback_lateral_thrust(self, msg):
        self.control_effort_lateral_thrust = msg.data
        self.set_lateral_thrust(self.control_effort_lateral_thrust)

    def control_callback_yaw(self, msg):
        self.control_effort_yaw = msg.data
        self.set_yaw_rate(self.control_effort_yaw)

    # def control_callback_pitch(self, msg):
    #     self.control_effort_pitch = 0  # msg.data
    #     self.set_pitch_rate(self.control_effort_pitch)

    # def control_callback_roll(self, msg):
    #     self.control_effort_roll = 0  # msg.data
    #     self.set_roll_rate(self.control_effort_roll)

    def set_thrust(self, value):
        self.thrust = max(-1, min(1, value))

    def set_yaw_rate(self, value):
        self.yaw_rate = max(-1, min(1, value))

    def set_vertical_thrust(self, value):
        self.vertical_thrust = max(-1, min(1, value))

    def set_lateral_thrust(self, value):
        self.lateral_thrust = max(-1, min(1, value))

    # def set_pitch_rate(self, value):
    #     self.pitch_rate = max(-1, min(1, value))

    # def set_roll_rate(self, value):
    #     self.roll_rate = max(-1, min(1, value))

    def publish_message(self):
        msg = ActuatorCommands()
        msg.header.stamp = rospy.Time.now()
        msg.thrust = self.thrust
        msg.yaw = self.yaw_rate
        msg.lateral_thrust = self.lateral_thrust
        msg.vertical_thrust = self.vertical_thrust
        msg.pitch = self.pitch_rate
        msg.roll = self.roll_rate
        self.actuator_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.publish_message()
            rate.sleep()


def main():
    node = Controller("main_controller")
    node.run()


if __name__ == "__main__":
    main()
