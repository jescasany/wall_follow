#!/usr/bin/env python  

"""
ROS node which subscribes to the /scan topic and fits a single line to
this scan, publishing it on the /extracted_lines topic.
"""
import pdb
#import roslib
#roslib.load_manifest('comp4766_a3_mod')
import rospy
import math
from fit_line import fit_line

from sensor_msgs.msg import LaserScan
from comp4766_a3_mod.msg import ExtractedLine
from comp4766_a3_mod.msg import ExtractedLines

def scan_callback(scan):
    """
    Fit a single line to the given laser scan.

    The line is fitted using scan points that are within 'maximum_range'
    distance (this parameter should be found by the ROS parameter server).  An
    ExtractedLines message is constructed to hold this single line and this
    message is published on /extracted_lines.
    """
    rospy.loginfo("START scan_callback")
    pdb.set_trace()
    # The fit_line method expects the scan itself, a pair of integers
    # describing the indices upon which it will operate, and the max. range.
    n = len(scan.ranges)
    line = fit_line(scan, 0, n-1, maximum_range)

    lines = ExtractedLines()
    lines.header.frame_id = '/laser'

    if line is not None:
        lines.lines.append(line)

    extracted_publisher.publish(lines)
    rospy.loginfo("STOP scan_callback")

if __name__ == '__main__':
    global maximum_range, extracted_publisher

    # Initialize this node.
    rospy.init_node('extract_single_line')

    maximum_range = rospy.get_param('maximum_range')

    # Subscribe to /scan
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    extracted_publisher = rospy.Publisher('/extracted_lines', ExtractedLines, queue_size=10)

    rospy.spin()
