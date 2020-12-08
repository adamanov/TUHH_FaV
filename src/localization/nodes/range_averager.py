#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
# from std_msgs.msg import Float64
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement
from depth_controller.msg import Orientation
from tf.transformations import euler_matrix
from localization.msg import RangesForPlotting

class RangeAverager():
    def __init__(self, name):
        rospy.init_node(name)

        self.range_past_values = [[],[],[],[]]
        self.nr_of_past_values = 1
        rospy.set_param('nr_average', 2)
        
        self.pub_range = rospy.Publisher("ranges/average", RangeMeasurementArray, queue_size=1)

        self.pub_range_for_plotting = rospy.Publisher("ranges/plotting", RangesForPlotting, queue_size=1)

        self.pub_range_for_plotting_average = rospy.Publisher("ranges/plotting/average", RangesForPlotting, queue_size=1)

        rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback)

    def ranges_for_plotting(self, msg, Publisher):
        ranges = RangesForPlotting()
        ranges.tag1, ranges.tag2, ranges.tag3, ranges.tag4 = 0.0, 0.0, 0.0, 0.0
        for measurement in msg.measurements:
            if measurement.id == 1:
                ranges.tag1 = measurement.range
            elif measurement.id == 2:
                ranges.tag2 = measurement.range
            elif measurement.id == 3:
                ranges.tag3 = measurement.range
            elif measurement.id == 4:
                ranges.tag4 = measurement.range
        Publisher.publish(ranges)

    def range_callback(self, msg):
        self.nr_of_past_values = rospy.get_param('nr_average')
        self.ranges_for_plotting(msg, self.pub_range_for_plotting)
        ids_of_tags = {0, 1, 2, 3}
        for measurement in msg.measurements:
            # print("range past values: " + str(self.range_past_values))
            current_id = measurement.id - 1
            ids_of_tags.discard(current_id)
            if len(self.range_past_values[current_id]) < self.nr_of_past_values:
                self.range_past_values[current_id].append(measurement.range)
            else:
                self.range_past_values[current_id].append(measurement.range)
                self.range_past_values[current_id].pop(0)
        for current_id in ids_of_tags:
            if len(self.range_past_values[current_id]) > 0:
                self.range_past_values[current_id].pop(0)
        # print("Past range values: " + str(self.range_past_values))
        self.average_past_range_values(msg)

    def average_past_range_values(self, msg):
        range_array = []
        # temp_string = ""
        for i in range(4):
            if len(self.range_past_values[i]) > 0:
                average = sum(self.range_past_values[i]) / len(self.range_past_values[i])
                temp_range = RangeMeasurement()
                temp_range.id = i + 1
                temp_range.range = average
                range_array.append(temp_range)
                # temp_string = temp_string + str(temp_range.id) + ": " + str(temp_range.range) + "   " 
        # print(temp_string)
        msg = RangeMeasurementArray()
        msg.measurements = range_array
        self.pub_range.publish(msg)
        self.ranges_for_plotting(msg, self.pub_range_for_plotting_average)

def main():
    node = RangeAverager("RangeAverager")
    rospy.spin()

if __name__ == "__main__":
   main()
