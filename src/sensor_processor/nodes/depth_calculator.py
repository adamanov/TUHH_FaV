#!/usr/bin/env python
import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64

class DepthCalculator():
   def __init__(self, name):
      rospy.init_node(name)
      
      self.pascal_per_meter = 1.0e4
      self.depth_offset = -0.15

      rospy.set_param('atmospheric_pressure', 1.004 * 10**5)  # [pascal]

      self.pastDepthValues = [0.0, 0.0, 0.0, 0.0]
      
      # PUBLISHER:
      self.depth_pub = rospy.Publisher("depth/state", Float64, queue_size=1)

      # SUBSCRIBER:
      rospy.Subscriber("pressure", FluidPressure, self.pressure_callback)

   def pressure_callback(self, pressure_msg):
      depth = -(pressure_msg.fluid_pressure - rospy.get_param('atmospheric_pressure')) / self.pascal_per_meter # + self.depth_offset
      self.pastDepthValues.append(depth)
      self.pastDepthValues.pop(0)
      depth_msg = Float64()
      depth_msg.data = sum(self.pastDepthValues)/len(self.pastDepthValues)
      self.depth_pub.publish(depth_msg)

def main():
   node = DepthCalculator("depth_calculator")
   rospy.spin()


if __name__ == "__main__":
   main()
