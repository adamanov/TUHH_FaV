#!/usr/bin/env python
import rospy
from bluerov_sim.msg import ActuatorCommands
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float64, Float32
from depth_controller.msg import Orientation
import numpy as np
import time
import math as m

class controller:
    def __init__(self,name):
        self.init_node(name)

        rospy.Subscriber("depth/setpoint",
                         Float64,
                         self.depth_setpoint_callback,
                         queue_size=1)

    




        
if __name__ == "__main__":
    node = controller("main_controller")
    node.run()
