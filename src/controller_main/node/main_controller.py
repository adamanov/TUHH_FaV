#!/usr/bin/env python
import rospy
from bluerov_sim.msg import ActuatorCommands
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float64, Float32
from depth_controller.msg import Orientation
from controller_main import currentPose
import numpy as np
import time
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs import Point


class Controller():
    def __init__(self, name):
        rospy.init_node(name)

        self.last_depth_time = 0

        rospy.set_param('depth_timeout', 1 * 10**9)

        self.rate = 30.0

        # self.arm_vehicle()

        self.euler = Orientation()

        rospy.set_param('euler_threshold', 10 * 2*m.pi/360)

        self.depth_setpoint = -0.5
        self.depth = 0.0

        # comming from desired/setpoint topic
        self.thrust_setpoint = 0.0
        # comming from desired/setpoint topic
        self.lateral_thrust_setpoint = 0.0
        # comming from desired/setpoint topic
        self.vertical_thrust_setpoint = 0.0

        self.yaw_rate = 0.0       # Desired is 0
        self.pitch_rate = 0.0     # Desired is 0
        self.roll_rate = 0.0      # Desired is 0

        self.actuator_pub = rospy.Publisher("mixer/actuator_commands",
                                            ActuatorCommands,
                                            queue_size=1)

        # Need create a desired setpoints Publisher (like depth_setpoint.py for all poses
        #  desired orientation can be also Published)
        rospy.Subscriber("desired_pose/setpoint",
                         Point,
                         self.desired_Pose_setpoint_callback,
                         queue_size=1)

        rospy.Subscriber("desired_angle/setpoint",
                         Point,
                         self.desired_Angle_setpoint_callback,
                         queue_size=1)

        # Subscribe to topics to get a current Pose and Orientation
        rospy.Subscriber("ground_truth/state", Odometry,
                         self.locationOrientation_callback, queue_size=1)
        rospy.Subscriber("localization/position_estimate",
                         Point, self.locationPose_callback, queue_size=1)

        # Subscribe to topics of the controller output
        rospy.Subscriber("thrust/control_effort",
                         Float64,
                         self.control_callback_thrust,
                         queue_size=1)
        rospy.Subscriber("vertical_thrust/control_effort",
                         Float64,
                         self.control_callback_vertical_thrust,
                         queue_size=1)
        rospy.Subscriber("lateral_thrust/control_effort",
                         Float64,
                         self.control_callback_lateral_thrust,
                         queue_size=1)
        rospy.Subscriber("yaw/control_effort",
                         Float64,
                         self.control_callback_yaw,
                         queue_size=1)
        rospy.Subscriber("pitch/control_effort",
                         Float64,
                         self.control_callback_pitch,
                         queue_size=1)
        rospy.Subscriber("roll/control_effort",
                         Float64,
                         self.control_callback_roll,
                         queue_size=1)

        rospy.Subscriber("orientation/euler", Orientation,
                         self.orient_callback,
                         queue_size=1)

# ------------------------control_callback_vertical_thrust-------------------------------------------------------
    def control_callback_vertical_thrust(self, msg):
        self.control_effort_vertical_thrust = msg.data
        safezone_upper = rospy.get_param('safezone_upper')
        safezone_lower = rospy.get_param('safezone_lower')
        # print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort))
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            if (self.depth < safezone_upper and self.depth > safezone_lower):
                # Congratulations You passed ALL CHECKS !!!
                self.set_vertical_thrust(self.control_effort_vertical_thrust)
            elif self.depth > safezone_upper:
                self.set_vertical_thrust(
                    min(self.control_effort_vertical_thrust, 0))
                rospy.loginfo(
                    "over safezone_upper!!   (No positive control_effort allowed)")
            elif self.depth < safezone_lower:
                self.set_vertical_thrust(max(self.control_effort, 0))
                rospy.loginfo(
                    "under safezone_lower!!   (No negative control_effort allowed)")
            else:
                self.set_vertical_thrust(0.0)
                rospy.loginfo(
                    "WARNING: Numerical Problems??? control_effort is 0!")
        elif isNotTimedout:
            self.set_vertical_thrust(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_vertical_thrust(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_vertical_thrust(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didn't recieved any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")


# ------------------------control_callback_thrust-------------------------------------------------------

    def control_callback_thrust(self, msg):
        self.control_effort_thrust = msg.data
        # print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort_thrust))
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            # Congratulations You passed ALL CHECKS !!!
            self.set_thrust(self.control_effort_thrust)
        elif isNotTimedout:
            self.set_thrust(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_thrust(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_thrust(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")


# --------------------control_callback_lateral_thrust--------------------------------------------------------------------

    def control_callback_lateral_thrust(self, msg):
        self.control_effort_lateral_thrust = msg.data
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            # Congratulations You passed ALL CHECKS !!!
            self.set_lateral_thrust(self.control_effort_lateral_thrust)
        elif isNotTimedout:
            self.set_lateral_thrust(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_lateral_thrust(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_lateral_thrust(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")


# -------------------------control_callback_yaw--------------------------------------------------------

    def control_callback_yaw(self, msg):
        self.control_effort_yaw = msg.data
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            # Congratulations You passed ALL CHECKS !!!
            self.set_yaw_rate(self.control_effort_yaw)
        elif isNotTimedout:
            self.set_yaw_rate(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_yaw_rate(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_yaw_rate(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")


# ---------------------control_callback_pitch--------------------------------------------------------------

    def control_callback_pitch(self, msg):
        self.control_effort_pitch = msg.data
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            # Congratulations You passed ALL CHECKS !!!
            self.set_pitch_rate(self.control_effort_pitch)
        elif isNotTimedout:
            self.set_pitch_rate(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_pitch_rate(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_pitch_rate(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")


# -----------------------control_callback_rol---------------------------------------------------------------


    def control_callback_roll(self, msg):
        self.control_effort_roll = msg.data
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            self.set_roll_rate(self.control_effort_roll)
            # rospy.loginfo("No limitations to control_effort_roll")
        elif isNotTimedout:
            self.set_roll_rate(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_roll_rate(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_roll_rate(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")


# ----- Reading and storing the comming msg. e.g ground_truth (yaw,roll,pitch) , localization (x,y,z)
# ----- and desired setpoint positions of robot, and after that separate each state and send set into PID
# ----- controller as a state and setpoint


    def locationPose_callback(self, CurrentPose):
        print(CurrentPose)
        Pose = Point()
        Pose.header.stamp = rospy.Time.now()
        # I am not sure about order x y z to thrust / vertical thrust / later thrust
        self.Pose.x = CurrentPose.x
        self.Pose.y = CurrentPose.y
        self.Pose.z = CurrentPose.z

        # Each Pose has to be published individually in order to PID Controller could subscribe

        # Publish current Thrust Position
        thrust_pub = rospy.Publisher(
            "thrust/state", Float64, queue_size=1)
        thrust_msg = Float64()
        thrust_msg.data = self.Pose.x
        thrust_pub.publish(thrust_msg)

        # Publish current Vertical Thrust Position
        vt_pub = rospy.Publisher(
            "vertical_thrust/state", Float64, queue_size=1)
        vt_msg = Float64()
        vt_msg.data = self.Pose.z
        vt_pub.publish(vt_msg)

        # Publish current Laterial Thrust Position
        lt_pub = rospy.Publisher(
            "lateral_thrust/state", Float64, queue_size=1)
        lt_msg = Float64()
        vt_msg.data = self.Pose.y
        lt_pub.publish(lt_msg)

    def locationOrientation_callback(self, CurrentOrienttion):
        print(CurrentOrienttion.pose.pose)
        print(CurrentOrienttion.pose.pose.Orientation)
        print(CurrentOrienttion.twist.twist.angular)

        # I am not sure about order angels
        self.roll = CurrentOrienttion.twist.twist.angular.x
        self.pitch = CurrentOrienttion.twist.twist.angular.y
        self.yaw = CurrentOrienttion.twist.twist.angular.z

        # Publish current Thrust Angle
        roll_pub = rospy.Publisher(
            "roll/state", Float64, queue_size=1)
        roll_msg = Float64()
        roll_msg.data = self.roll
        roll_pub.publish(roll_msg)

        # Publish current Pitch Angle
        pitch_pub = rospy.Publisher(
            "roll/state", Float64, queue_size=1)
        pitch_msg = Float64()
        pitch_msg.data = self.pitch
        pitch_pub.publish(pitch_msg)

        # Publish current Yaw Angle
        yaw_pub = rospy.Publisher("yaw/state", Float64, queue_size=1)
        yaw_msg = Float64()
        yaw_msg = self.yaw
        yaw_pub.publish(yaw_msg)

       # Each Angle has to be published individually in order to PID Controller could subscribe

    def desired_Pose_setpoint_callback(self, desiredPose):
        print(desiredPose)
        self.thrust_setpoint = desiredPose.x
        self.lateral_thrust_setpoint = desiredPose.y
        self.vertical_thrust_setpoint = desiredPose.z

        # Each Pose Setpoint has to be published individually in order to PID Controller could subscribe

        # Publish setpoints Thrust Position for a controller
        thrust_setpoint_pub = rospy.Publisher(
            "thrust/setpoint", Float64, queue_size=1)
        thrust_msg = Float64()
        thrust_msg.data = self.thrust_setpoint
        thrust_setpoint_pub.publish(thrust_msg)

        # Publish setpoints Vertical Thrust Position for a controller
        vt_setpoint_pub = rospy.Publisher(
            "vertical_thrust/setpoint", Float64, queue_size=1)
        vt_msg = Float64()
        vt_msg = self.vertical_thrust_setpoint
        vt_setpoint_pub.publish(vt_msg)
        # Publish setpoints laterial Position for a controller
        lt_setpoint_pub = rospy.Publisher(
            "lateral_thrust/setpoint", Float64, queue_size=1)
        lt_msg = Float64()
        lt_msg = self.lateral_thrust_setpoint
        lt_setpoint_pub.publish(lt_msg)

    def desired_Angle_setpoint_callback(self, desiredOrientation):
        print(desiredOrientation)
        self.yaw_rate = desiredOrientation.x
        self.pitch_rate = desiredOrientation.y
        self.roll_rate = desiredOrientation.z

        # Publish setpoints Yaw Angle for a controller
        yaw_setpoint_pub = rospy.Publisher(
            "yaw/setpoint", Float64, queue_size=1)
        yaw_msg = Float64()
        yaw_msg = self.yaw_rate
        yaw_setpoint_pub.publish(yaw_msg)
        # Publish setpoints Pitch Angle for a controller
        pitch_setpoint_pub = rospy.Publisher(
            "pitch/setpoint", Float64, queue_size=1)
        pitch_msg = Float64()
        pitch_msg = self.pitch_rate
        pitch_setpoint_pub.publish(pitch_msg)

        # Publish setpoints Roll Angle for a controller
        roll_setpoint_pub = rospy.Publisher(
            "roll/setpoint", Float64, queue_size=1)
        roll_msg = Float64()
        roll_msg = self.roll_rate
        roll_setpoint_pub.publish(roll_msg)


# --- PID Controller required things are done


    def orient_callback(self, msg):
        self.euler = msg

    def isStable(self):
        return (rospy.get_param('euler_threshold') > max(abs(self.euler.roll), abs(self.euler.pitch)))

    def arm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        while not arm(True).success:
            rospy.logwarn_throttle(1, "Could not arm vehicle. Keep trying.")
        rospy.loginfo("Armed successfully.")

    def disarm_vehicle(self):
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        arm(False)

    def set_thrust(self, value):
        self.thrust = max(-1, min(1, value))

    def set_yaw_rate(self, value):
        self.yaw_rate = max(-1, min(1, value))

    def set_vertical_thrust(self, value):
        self.vertical_thrust = max(-1, min(1, value))

    def set_lateral_thrust(self, value):
        self.lateral_thrust = max(-1, min(1, value))

    def set_pitch_rate(self, value):
        self.pitch_rate = max(-1, min(1, value))

    def set_roll_rate(self, value):
        self.roll_rate = max(-1, min(1, value))

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
    print(" we could")
    node = controllerClass("main_controller")
    node.run()


if __name__ == "__main__":
    main()
