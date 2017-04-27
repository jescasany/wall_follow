#!/usr/bin/env python  

"""
ROS node which subscribes to the /base_scan topic and publishes a filtered
laser scan on /scan.  Unusable values such as 'inf' and 'nan' in /base_scan
are converted to maximum_range + 1 in /scan.
"""

#import roslib
#roslib.load_manifest('comp4766_a3_mod')
import rospy
from copy import deepcopy
from math import isnan

from sensor_msgs.msg import LaserScan

def base_scan_callback(base_scan):
    scan = deepcopy(base_scan)

    scan.ranges = []
    for i in range(len(base_scan.ranges)):
        rho = base_scan.ranges[i]
        if rho == float('inf') or isnan(rho):
            scan.ranges.append(maximum_range + 1)
        else:
            scan.ranges.append(rho)

    publisher.publish(scan)

if __name__ == '__main__':
    global maximum_range, publisher

    # Initialize this node.
    rospy.init_node('laser_filter') 
    maximum_range = rospy.get_param('maximum_range')

    # Subscribe to /base_scan
    rospy.Subscriber('/base_scan', LaserScan, base_scan_callback)

    publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)

    rospy.spin()
