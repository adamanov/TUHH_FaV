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
        


def main():
    print(" we could")
    node = controllerClass("main_controller")
    node.run()

    


if __name__ == "__main__":
    main()
