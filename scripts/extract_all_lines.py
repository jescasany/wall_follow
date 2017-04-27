#!/usr/bin/env python  

"""
A modified version of the solution to Assignment 3, Part 2 in COMP 4766/6778,
Winter 2014.  Overall, this package has been modified from the assignment
solution in the following ways:
    - Incorporated laser_filter.py which eliminates data points with infinite
      or invalid range values.  That file implements a ROS node which subscribes
      to /base_scan and publishes /scan.
    - Using a modifed version of the split-and-merge algorithm to better handle
      the presence of multiple walls by selecting the worst-fit point to be a
      local maximum in orthogonal line distance..
    - Added a constraint on the line fitting procedure to prevent negative
      r values.

ROS node which subscribes to the /scan topic and applies the
split-and-merge algorithm to fit multiple lines to this data.  These lines are
published on the /extracted_lines topic.
"""

import roslib
#roslib.load_manifest('comp4766_a3_mod')
import rospy
import math
from fit_line import fit_line

from sensor_msgs.msg import LaserScan
from comp4766_a3_mod.msg import ExtractedLine
from comp4766_a3_mod.msg import ExtractedLines

# Set to True to log debug information
debug = False

def scan_callback(scan):
    """
    Extracts lines from the given LaserScan and publishes to /extracted_lines.

    Extracts lines using the split phase of the split-and-merge algorithm.
    Publish these lines as an ExtractedLines method on the /extracted_lines
    topic.
    """

    # Create an ExtractedLines method and initialize some fields in the header.
    lines = ExtractedLines()
    lines.header.frame_id = '/laser'
    #lines.header.frame_id = '/base_laser_link'
    lines.header.stamp = rospy.Time.now()

    # Create our big list of index pairs.  Each pair gives the start and end
    # index which specifies a contiguous set of data points in 'scan'.
    L = []

    # Place (0, n-1) in the list which represents the complete list of scan
    # points.
    n = len(scan.ranges)
    L.append((0, n-1))

    # We'll keep track of the iteration count with 'I' just for debug purposes.
    I = 0

    # The big loop.  At each step we process the sublist stored in the first
    # pair of indices in L.
    while len(L) != 0:
        if debug: rospy.loginfo( "I: " + str(I))
        I += 1

        # Get the first sublist, i will refer to the starting index and j
        # the final index.
        (i, j) = L.pop(0)
        if debug: 
            rospy.loginfo('Processing: ({}, {})'.format(i, j))

        line = fit_line(scan, i, j, maximum_range)

        if line is None:
            if debug: rospy.loginfo('\tNo line found')
            continue

        if debug: rospy.loginfo('\tLine: ({}, {})'.format(line.r, line.alpha))

        # Find the worst fit point.  We will consider a point to be the
        # worst fit if it represents a local maximum in distance from the line.
        # That is, its absolute distance to the line must be greater than that
        # of its two neighbours.  First compute an array of distances.  We will
        # not consider the first or last min_points_per_line / 2 
        # points.  So the length of this array is the following:
        nDistances = j - i + 1 - min_points_per_line
        distances = [0] * nDistances # Fill with zeros
        mppl_2 = min_points_per_line / 2
        # distIndex is the index into distances
        # laserIndex is the index into the laser scan
        for distIndex in range(nDistances):
            laserIndex = i + mppl_2 + distIndex
            # Compute the orthogonal distance to the line, d
            rho = scan.ranges[laserIndex]
            theta = scan.angle_min + laserIndex * scan.angle_increment
            d = abs(rho * math.cos(theta - line.alpha) - line.r)
            distances[distIndex] = d
        # Now check for the worst fit point using these distances.
        worstLaserIndex = -1 
        worstDistance = 0
        for laserIndex in range(i + mppl_2 + 1, j - mppl_2 - 1):
            if scan.ranges[laserIndex] > maximum_range:
                continue
            distIndex = laserIndex - i - mppl_2
            if distances[distIndex] >= distances[distIndex - 1] and \
               distances[distIndex] >= distances[distIndex + 1] and \
               distances[distIndex] > worstDistance:
                worstLaserIndex = laserIndex
                worstDistance = distances[distIndex]
        if debug:
            rospy.loginfo( "\tworstLaserIndex: " + str(worstLaserIndex))
            rospy.loginfo( "\tworstDistance: " + str(worstDistance))

        if worstDistance < orthog_distance_threshold:
            # We have dealt with this sublist.  Add it to lines.
            if debug: rospy.loginfo('\tAdding line')
            lines.lines.append(line)
        else:
            # Split this sublist into two and add both back into L.
            if debug: 
                rospy.loginfo('\tSplitting: ({}, {})'.format(i, j))
            insert_sublist_if_long(L, 0, (i, worstLaserIndex-1))
            insert_sublist_if_long(L, 1, (worstLaserIndex+1, j))

    if debug: rospy.loginfo( "\n\n")
    
    # Normally the merge step would follow...

    extracted_publisher.publish(lines)

def insert_sublist_if_long(L, insert_position, (start_index, stop_index)):
    """Insert the given index pair into L if it is sufficiently long."""

    if stop_index - start_index + 1 > min_points_per_line:
        if debug: 
            rospy.loginfo('Inserting: ({}, {})'.format(start_index, stop_index))
        L.insert(insert_position, (start_index, stop_index))

if __name__ == '__main__':
    global orthog_distance_threshold, min_points_per_line, maximum_range, \
           extracted_publisher

    # Read parameters managed by the ROS parameter server
    orthog_distance_threshold = rospy.get_param('orthog_distance_threshold')
    min_points_per_line = rospy.get_param('min_points_per_line')
    maximum_range = rospy.get_param('maximum_range')

    # The min_points_per_line parameter should be an even number greater than 0.
    assert min_points_per_line % 2 == 0
    assert min_points_per_line > 0

    # Initialize this node.
    rospy.init_node('extract_all_lines')

    # Subscribe to /scan
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Create our publisher for the lines extracted within 'scan_callback'
    extracted_publisher = rospy.Publisher('/extracted_lines', ExtractedLines, queue_size=10)

    rospy.spin()
