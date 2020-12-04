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


class controllerClass:
    def __init__(self, name):
        rospy.init_node(name)
        

class Controller():
    def __init__(self, name):
        rospy.init_node(name)
        
        self.last_depth_time = 0

        rospy.set_param('depth_timeout', 1 * 10**9)

        self.rate = 30.0

        # self.arm_vehicle()

        self.euler = Orientation()

        rospy.set_param('euler_threshold', 10* 2*m.pi/360)

        self.depth_setpoint = -0.5
        self.depth = 0.0

        self.thrust = 0.0
        self.lateral_thrust = 0.0
        self.vertical_thrust = 0.0
        self.yaw_rate = 0.0
        self.pitch_rate = 0.0
        self.roll_rate = 0.0


        self.actuator_pub = rospy.Publisher("mixer/actuator_commands",
                                            ActuatorCommands,
                                            queue_size=1)

        rospy.Subscriber("depth/setpoint",
                         Float64,
                         self.depth_setpoint_callback,
                         queue_size=1)

        rospy.Subscriber("depth/state",
                         Float64,
                         self.depth_callback,
                         queue_size=1)

        rospy.Subscriber("depth/control_effort_thrust",
                         Float64,
                         self.control_callback_thrust,
                         queue_size=1)

        rospy.Subscriber("depth/control_effort_vertical_thrust",
                         Float64,
                         self.control_callback_vertical_thrust,
                         queue_size=1)  
        rospy.Subscriber("depth/control_effort_lateral_thrust",
                         Float64,
                         self.control_callback_lateral_thrust,
                         queue_size=1)                      
        rospy.Subscriber("depth/control_effort_yaw",
                        Float64,
                        self.control_callback_yaw,
                        queue_size=1)          
        rospy.Subscriber("depth/control_effort_pitch",
                         Float64,
                         self.control_callback_pitch,
                         queue_size=1)                                     
        rospy.Subscriber("depth/control_effort_roll",
                         Float64,
                         self.control_callback_roll,
                         queue_size=1)  

        rospy.Subscriber("orientation/euler", Orientation,
                         self.orient_callback,
                         queue_size=1)

    def control_callback_vertical_thrust(self, msg):
        self.control_effort_vertical_thrust = msg.data
        safezone_upper = rospy.get_param('safezone_upper')
        safezone_lower = rospy.get_param('safezone_lower')
        #print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort))
        isNotTimedout = rospy.get_param('depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            if (self.depth < safezone_upper and self.depth > safezone_lower):
                self.set_vertical_thrust(self.control_effort_vertical_thrust)
                # rospy.loginfo("No limitations to control_effort")
            elif self.depth > safezone_upper:
                self.set_vertical_thrust(min(self.control_effort, 0))
                rospy.loginfo("over safezone_upper!!   (No positive control_effort allowed)")
            elif self.depth < safezone_lower:
                self.set_vertical_thrust(max(self.control_effort, 0))
                rospy.loginfo("under safezone_lower!!   (No negative control_effort allowed)")
            else:
                self.set_vertical_thrust(0.0)
                rospy.loginfo("WARNING: Numerical Problems??? control_effort is 0!")
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
            rospy.loginfo("Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo("(You could change 'depth_timeout' and 'euler_threshold')")

#------------------------------------------------------------------------------------------------------

def control_callback_thrust(self, msg):
        self.control_effort_thrust = msg.data
        safezone_upper = rospy.get_param('safezone_upper')
        safezone_lower = rospy.get_param('safezone_lower')
        #print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort))
        isNotTimedout = rospy.get_param('depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            if (self.depth < safezone_upper and self.depth > safezone_lower):
                self.set_thrust(self.control_effort)
                # rospy.loginfo("No limitations to control_effort")
            elif self.depth > safezone_upper:
                self.set_thrust(min(self.control_effort, 0))
                rospy.loginfo("over safezone_upper!!   (No positive control_effort allowed)")
            elif self.depth < safezone_lower:
                self.set_thrust(max(self.control_effort, 0))
                rospy.loginfo("under safezone_lower!!   (No negative control_effort allowed)")
            else:
                self.set_thrust(0.0)
                rospy.loginfo("WARNING: Numerical Problems??? control_effort is 0!")
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
            rospy.loginfo("Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo("(You could change 'depth_timeout' and 'euler_threshold')")

#------------------------------------------------------------------------------------------------------

def control_callback_lateral_thrust(self, msg):
        self.control_effort_lateral_thrust = msg.data
        safezone_upper = rospy.get_param('safezone_upper')
        safezone_lower = rospy.get_param('safezone_lower')
        #print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort))
        isNotTimedout = rospy.get_param('depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            if (self.depth < safezone_upper and self.depth > safezone_lower):
                self.set_lateral_thrust(self.control_effort)
                # rospy.loginfo("No limitations to control_effort")
            elif self.depth > safezone_upper:
                self.set_lateral_thrust(min(self.control_effort, 0))
                rospy.loginfo("over safezone_upper!!   (No positive control_effort allowed)")
            elif self.depth < safezone_lower:
                self.set_lateral_thrust(max(self.control_effort, 0))
                rospy.loginfo("under safezone_lower!!   (No negative control_effort allowed)")
            else:
                self.set_lateral_thrust(0.0)
                rospy.loginfo("WARNING: Numerical Problems??? control_effort is 0!")
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
            rospy.loginfo("Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo("(You could change 'depth_timeout' and 'euler_threshold')")            

#------------------------------------------------------------------------------------------------------

def control_callback_yaw(self, msg):
        self.control_effort_yaw = msg.data
        safezone_upper = rospy.get_param('safezone_upper')
        safezone_lower = rospy.get_param('safezone_lower')
        #print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort))
        isNotTimedout = rospy.get_param('depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            if (self.depth < safezone_upper and self.depth > safezone_lower):
                self.set_yaw_rate(self.control_effort)
                # rospy.loginfo("No limitations to control_effort")
            elif self.depth > safezone_upper:
                self.set_yaw_rate(min(self.control_effort, 0))
                rospy.loginfo("over safezone_upper!!   (No positive control_effort allowed)")
            elif self.depth < safezone_lower:
                self.set_yaw_rate(max(self.control_effort, 0))
                rospy.loginfo("under safezone_lower!!   (No negative control_effort allowed)")
            else:
                self.set_yaw_rate(0.0)
                rospy.loginfo("WARNING: Numerical Problems??? control_effort is 0!")
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
            rospy.loginfo("Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo("(You could change 'depth_timeout' and 'euler_threshold')")


#------------------------------------------------------------------------------------------------------

def control_callback_pitch(self, msg):
        self.control_effort_pitch = msg.data
        safezone_upper = rospy.get_param('safezone_upper')
        safezone_lower = rospy.get_param('safezone_lower')
        #print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort))
        isNotTimedout = rospy.get_param('depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            if (self.depth < safezone_upper and self.depth > safezone_lower):
                self.set_pitch_rate(self.control_effort)
                # rospy.loginfo("No limitations to control_effort")
            elif self.depth > safezone_upper:
                self.set_pitch_rate(min(self.control_effort, 0))
                rospy.loginfo("over safezone_upper!!   (No positive control_effort allowed)")
            elif self.depth < safezone_lower:
                self.set_pitch_rate(max(self.control_effort, 0))
                rospy.loginfo("under safezone_lower!!   (No negative control_effort allowed)")
            else:
                self.set_pitch_rate(0.0)
                rospy.loginfo("WARNING: Numerical Problems??? control_effort is 0!")
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
            rospy.loginfo("Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo("(You could change 'depth_timeout' and 'euler_threshold')")


#------------------------------------------------------------------------------------------------------

def control_callback_roll(self, msg):
        self.control_effort_roll = msg.data
        safezone_upper = rospy.get_param('safezone_upper')
        safezone_lower = rospy.get_param('safezone_lower')
        #print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort))
        isNotTimedout = rospy.get_param('depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            if (self.depth < safezone_upper and self.depth > safezone_lower):
                self.set_roll_rate(self.control_effort)
                # rospy.loginfo("No limitations to control_effort")
            elif self.depth > safezone_upper:
                self.set_roll_rate(min(self.control_effort, 0))
                rospy.loginfo("over safezone_upper!!   (No positive control_effort allowed)")
            elif self.depth < safezone_lower:
                self.set_roll_rate(max(self.control_effort, 0))
                rospy.loginfo("under safezone_lower!!   (No negative control_effort allowed)")
            else:
                self.set_roll_rate(0.0)
                rospy.loginfo("WARNING: Numerical Problems??? control_effort is 0!")
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
            rospy.loginfo("Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo("(You could change 'depth_timeout' and 'euler_threshold')")




    def depth_callback(self, msg):
        self.depth = msg.data
        self.last_depth = rospy.Time.now().nsecs
        # time.sleep(3)

    def depth_setpoint_callback(self, msg):
        if isinstance(msg.data, float):
            self.depth_setpoint = msg.data
        else:
            rospy.loginfo("Setpoint rejected: Setpoint has to be a float!")

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
         self.roll_rate = max(-1, min(1, value)))
    
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

def main():
    print(" we could")
    node = controllerClass("main_controller")
    node.run()

    


if __name__ == "__main__":
    main()
