#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from range_sensor.msg import RangeMeasurementArray

class PositionEstimator():
    def __init__(self, name):
        rospy.init_node(name)
        range_array = []
        depth = 0.0
        rospy.Subscriber("ranges", RangeMeasurementArray,
                                    self.range_callback)
        rospy.Subscriber("depth/state", Float64, self.depth_callback)
        # What message type do we need?
        rospy.Publisher("position", Point, queue_size=1)
                    
    def range_callback(self, msg):
        range_array = msg.measurements
    
    def depth_callback():

    def get_tag_coordinates(id):
        # Adjust these for the experiment!
        coordinates = [[0.5, 3.35, -0.5], [1.1, 3.35, -0.5],
                       [0.5, 3.35, -0.9], [1.1, 3.35, -0.9]]
        return coordinates[id]

    def estimate_current_position(self):
        num_tag_measurements = len(self.range_array)
        if num_tag_measurements == 0:
            print("hello")
        elif num_tag_measurements == 1 :

        elif num_tag_measurements == 2 :

        elif num_tag_measurements == 3 :

        elif num_tag_measurements == 4 :

def main():
    node = PositionEstimator("PositionEstimator")
    rospy.spin()

if __name__ == "__main__":
   main()
