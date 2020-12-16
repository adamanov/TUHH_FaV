#!/usr/bin/env python
import rospy
import numpy as np
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement
from sensor_processor.msg import RangesForPlotting

class RangeViewer():
    def __init__(self, name):
        rospy.init_node(name)

        # PUBLISHER:
        self.pub_range_for_plotting = rospy.Publisher("ranges/plotting", RangesForPlotting, queue_size=1)

        # SUBSCRIBER:
        rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback)

    def range_callback(self, msg):
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
        self.pub_range_for_plotting.publish(ranges)


def main():
    node = RangeViewer("RangeViewer")
    rospy.spin()

if __name__ == "__main__":
   main()
