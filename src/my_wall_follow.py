#!/usr/bin/env python 
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 27 12:50:35 2017

ROS node which subscribes to the /scan topic and fits a single line to
this scan, publishing it on the /extracted_lines topic.

ROS node which subscribes to the /base_scan topic and publishes a filtered
laser scan on /scan.  Unusable values such as 'inf' and 'nan' in /base_scan
are converted to maximum_range + 1 in /scan.

Fit a line to the given LaserScan yielding an ExtractedLine.

The method of weighted least squares is applied to yield a description of
the line consisting of the tuple (r, alpha) where 'r' is the orthogonal
distance of the line to the origin and 'alpha' is the angle that the ray
perpendicular to the line makes with the origin.  Weighted least squares is
applied as described in "Introduction to Autonomous Mobile Robots" by
Siegwart, Nourbakhsh, and Scaramuzza.

Arguments:
start_index -- index of the first data point to be used in the line fitting
               procedure.
end_index -- index of the last data point to be used in the line fitting
             procedure.
maximum_range -- specifies the maximum range of data points which will be
                 incorporated into the fitted line.

If the scan is empty or there are no points within 'maximum_range' then
None is returned.

This package has been modified from the assignment
solution in the following ways:
    - Incorporated laser_filter.py which eliminates data points with infinite
      or invalid range values.  That file implements a ROS node which subscribes
      to /base_scan and publishes /scan.
    - Using a modifed version of the split-and-merge algorithm to better handle
      the presence of multiple walls by selecting the worst-fit point to be a
      local maximum in orthogonal line distance.
    - Added a constraint on the line fitting procedure to prevent negative
      r values.

Extracted lines subscribes to the /scan topic and applies the
split-and-merge algorithm to fit multiple lines to this data.  These lines are
published on the /extracted_lines topic.

Display Lines displays lines extracted from laser range scans.  The node
subscribes to 'lines_topic' and publishes the data needed to display these
lines to 'vis_lines_topic' and correspondingly coloured first and last scan points used to generate each line to 'vis_scanpoints_topic'.  RViz must be
configured appropriately to display the messages posted to these topics.

@author: juan
"""

import pdb
#import roslib
#roslib.load_manifest('comp4766_a3_mod')
import rospy
import math
import copy

import sensor_msgs.msg
from comp4766_a3_mod.msg import ExtractedLine
from comp4766_a3_mod.msg import ExtractedLines

from geometry_msgs.msg import Point32
from angles import constrain_angle

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

""" A class to track black_board.variables """
class BlackBoard:
    def __init__(self):
        self.lines_topic = rospy.get_param('lines_topic')
        self.vis_lines_topic = rospy.get_param('vis_lines_topic')
        self.vis_scanpoints_topic = rospy.get_param('vis_scanpoints_topic')
        # Read parameters managed by the ROS parameter server
        self.orthog_distance_threshold = rospy.get_param('orthog_distance_threshold')
        self.min_points_per_line = rospy.get_param('min_points_per_line')
        self.maximum_range = rospy.get_param('maximum_range')
        
        self.scan = sensor_msgs.msg.LaserScan()
        
black_board = BlackBoard()
# The min_points_per_line parameter should be an even number greater than 0.
assert black_board.min_points_per_line % 2 == 0
assert black_board.min_points_per_line > 0

def fit_line(scan, start_index, end_index, maximum_range):
    # First we must calculate alpha, using the formula presented in the
    # course notes (due to Arras, 1998).
    sumWeights = 0
    sumNumL = 0 # The summation term in the numerator on the left
    sumNumR = 0 # The summation term in the numerator on the right
    sumDenL = 0 # The summation term in the denominator on the left
    sumDenR = 0 # The summation term in the denominator on the rt.
    for i in range(start_index, end_index+1):

        rho = scan.ranges[i]
        if rho == 0 or rho > maximum_range:
            continue
        theta = scan.angle_min + i * scan.angle_increment
        weight = 1 / rho**2
        #weight = 1
        sumWeights += weight

        factor1 = weight * rho * rho
        sumNumL += factor1 * math.sin(2 * theta)
        sumDenL += factor1 * math.cos(2 * theta)
        
        for j in range(start_index, end_index+1):
            rho_j = scan.ranges[j]
            if rho_j == 0 or rho_j > maximum_range:
                continue
            theta_j = scan.angle_min + j * scan.angle_increment
            weight_j = 1 / rho_j**2
            #weight_j = 1

            factor2 = weight * weight_j * rho * rho_j
            sumNumR += factor2 * math.cos(theta) * math.sin(theta_j)
            sumDenR += factor2 * math.cos(theta + theta_j)

    if sumWeights == 0:
        # There are either no scan points at all, or none within range.
        return None

    sumNumR *= 2.0 / sumWeights
    sumDenR /= sumWeights
    alpha = math.atan2(sumNumL - sumNumR, sumDenL - sumDenR) / 2.0 + math.pi/2

    # We now calculate r.
    sumNum = 0 # The summation term in the numerator
    for i in range(start_index, end_index+1):
        rho = scan.ranges[i]
        if rho == 0 or rho > maximum_range:
            continue
        theta = scan.angle_min + i * scan.angle_increment
        weight = 1 / rho**2
        #weight = 1

        sumNum += weight * rho * math.cos(theta - alpha)

    r = sumNum / sumWeights

    # It is possible that the r value returned above is negative.  We formulate
    # r as a positive quantity, but the optimization process doesn't know about
    # this.  Having a negative r can create problems down the road (e.g. for
    # line-based wall following).  So we flip our parameters to make r positive.
    if r < 0:
        r *= -1
        alpha += math.pi

    # Make sure that alpha is in the range (-pi, pi].
    alpha = constrain_angle(alpha)

    # Determine the first and last points used to estimate this line's
    # parameters.  These two points do not define the line, but they are useful
    # for visualization to show the range of points involved.
    firstScanPoint = Point32()
    lastScanPoint = Point32()
    dist = scan.ranges[start_index]
    angle = scan.angle_min + start_index * scan.angle_increment
    if dist <= maximum_range:
        firstScanPoint.x = dist * math.cos(angle)
        firstScanPoint.y = dist * math.sin(angle)
    dist = scan.ranges[end_index]
    angle = scan.angle_min + end_index * scan.angle_increment
    if dist <= maximum_range:
        lastScanPoint.x = dist * math.cos(angle)
        lastScanPoint.y = dist * math.sin(angle)

    return ExtractedLine(r, alpha, firstScanPoint, lastScanPoint)

# Set to True to log debug information
debug = False

def insert_sublist_if_long(L, insert_position, (start_index, stop_index)):
    """Insert the given index pair into L if it is sufficiently long."""
    
    if stop_index - start_index + 1 > black_board.min_points_per_line:
        if debug: 
            rospy.loginfo('Inserting: ({}, {})'.format(start_index, stop_index))
        L.insert(insert_position, (start_index, stop_index))
        
def create_lines_marker(lines):
    marker_lines = Marker()

    # Get the pairs of points that comprise the endpoints for all lines
    marker_lines.points = generate_endpoints(lines)

    # Initialize basic fields of the marker
    marker_lines.header.frame_id = lines.header.frame_id
    marker_lines.header.stamp = rospy.Time()
    marker_lines.id = 0
    marker_lines.type = Marker.LINE_LIST
    marker_lines.action = Marker.ADD

    # The marker's pose is at (0,0,0) with no rotation.
    marker_lines.pose.orientation.w = 1

    # Set line width
    marker_lines.scale.x = 0.05

    n = len(lines.lines)
    if n == 1:
        # There is only one line.  Just set the color field.
        marker_lines.color.r = 1.0
        marker_lines.color.a = 1.0
    if n > 1:
        # Set per-vertex colours
        for i in range(n):
            color = ColorRGBA()
            # A very simplistic colour spectrum.
            color.r = 1.0 - i/float(n-1)
            color.g = 1.0
            color.b = i/float(n-1)
            color.a = 1.0
            marker_lines.colors.append(color)
            marker_lines.colors.append(color)

    lines_publisher.publish(marker_lines)

def create_scanpoints_marker(lines):
    marker_points = Marker()

    # Initialize basic fields of the marker
    marker_points.header.frame_id = lines.header.frame_id
    marker_points.header.stamp = rospy.Time()
    marker_points.id = 0
    marker_points.type = Marker.POINTS
    marker_points.action = Marker.ADD

    # The marker's pose is at (0,0,0) with no rotation.
    marker_points.pose.orientation.w = 1

    # Set point width
    marker_points.scale.x = 0.10
    marker_points.scale.y = 0.10

    # Add the scan points
    marker_points.points = []
    n = len(lines.lines)
    for i in range(n):
        marker_points.points.append(lines.lines[i].firstScanPoint)
        marker_points.points.append(lines.lines[i].lastScanPoint)

    if n == 1:
        # There is only one line.  Just set the color field.
        marker_points.color.r = 1.0
        marker_points.color.a = 1.0
    if n > 1:
        # Set per-vertex colours
        for i in range(n):
            color = ColorRGBA()
            # A very simplistic colour spectrum.
            color.r = 1.0 - i/float(n-1)
            color.g = 1.0
            color.b = i/float(n-1)
            color.a = 1.0
            marker_points.colors.append(color)
            marker_points.colors.append(color)

    black_board.scanpoints_publisher.publish(marker_points)

def generate_endpoints(lines):
    """
    Generate two endpoints for each given line.

    Returns a list of point pairs to represent each of the given lines.  
    The points are positioned at a fixed distance 'q' from the point on the
    line closest to the origin (where the orthogonal ray hits the line).
    """
    pdb.set_trace()
    # Distance from the closest point on the line to the origin at which to
    # place the endpoints.
    q = 5

    # The list of points
    points = []

    n = len(lines.lines)
    for i in range(n):
        r = lines.lines[i].r
        alpha = lines.lines[i].alpha

        # Each point is a vector sum from the origin to the point on the line
        # closest to the origin + a vector along the line in either direction.
        point1 = Point(r*math.cos(alpha) + q*math.cos(alpha+math.pi/2), \
                       r*math.sin(alpha) + q*math.sin(alpha+math.pi/2), \
                       0)
        point2 = Point(r*math.cos(alpha) + q*math.cos(alpha-math.pi/2), \
                       r*math.sin(alpha) + q*math.sin(alpha-math.pi/2), \
                       0)
        points.append(point1)
        points.append(point2)
        print points
    return points        
        
def scan_callback(scan):
    """
    Fit a single line to the given laser scan.

    The line is fitted using scan points that are within 'maximum_range'
    distance (this parameter should be found by the ROS parameter server).  An
    ExtractedLines message is constructed to hold this single line and this
    message is published on /extracted_lines.
    """
    pdb.set_trace()
    rospy.loginfo("START scan_callback")
    print scan
    # The fit_line method expects the scan itself, a pair of integers
    # describing the indices upon which it will operate, and the max. range.
    n = len(scan.ranges)
    line = fit_line(scan, 0, n-1, black_board.maximum_range)
    print line
    lines = ExtractedLines()
    lines.header.frame_id = '/laser'
    
    if line is not None:
        lines.lines.append(line)
    print lines
    black_board.extracted_publisher.publish(lines)
    
#    """
#    Extracts lines from the given LaserScan and publishes to /extracted_lines.
#
#    Extracts lines using the split phase of the split-and-merge algorithm.
#    Publish these lines as an ExtractedLines method on the /extracted_lines
#    topic.
#    """
#    # Create an ExtractedLines method and initialize some fields in the header.
#    lines = ExtractedLines()
#    lines.header.frame_id = '/laser'
#    #lines.header.frame_id = '/base_laser_link'
#    lines.header.stamp = rospy.Time.now()
#
#    # Create our big list of index pairs.  Each pair gives the start and end
#    # index which specifies a contiguous set of data points in 'scan'.
#    L = []
#
#    # Place (0, n-1) in the list which represents the complete list of scan
#    # points.
#    n = len(scan.ranges)
#    L.append((0, n-1))
#
#    # We'll keep track of the iteration count with 'I' just for debug purposes.
#    I = 0
#
#    # The big loop.  At each step we process the sublist stored in the first
#    # pair of indices in L.
#    while len(L) != 0:
#        if debug: rospy.loginfo( "I: " + str(I))
#        I += 1
#
#        # Get the first sublist, i will refer to the starting index and j
#        # the final index.
#        (i, j) = L.pop(0)
#        if debug: 
#            rospy.loginfo('Processing: ({}, {})'.format(i, j))
#
#        line = fit_line(scan, i, j, maximum_range)
#
#        if line is None:
#            if debug: rospy.loginfo('\tNo line found')
#            continue
#
#        if debug: rospy.loginfo('\tLine: ({}, {})'.format(line.r, line.alpha))
#
#        # Find the worst fit point.  We will consider a point to be the
#        # worst fit if it represents a local maximum in distance from the line.
#        # That is, its absolute distance to the line must be greater than that
#        # of its two neighbours.  First compute an array of distances.  We will
#        # not consider the first or last min_points_per_line / 2 
#        # points.  So the length of this array is the following:
#        nDistances = j - i + 1 - min_points_per_line
#        distances = [0] * nDistances # Fill with zeros
#        mppl_2 = min_points_per_line / 2
#        # distIndex is the index into distances
#        # laserIndex is the index into the laser scan
#        for distIndex in range(nDistances):
#            laserIndex = i + mppl_2 + distIndex
#            # Compute the orthogonal distance to the line, d
#            rho = scan.ranges[laserIndex]
#            theta = scan.angle_min + laserIndex * scan.angle_increment
#            d = abs(rho * math.cos(theta - line.alpha) - line.r)
#            distances[distIndex] = d
#        # Now check for the worst fit point using these distances.
#        worstLaserIndex = -1 
#        worstDistance = 0
#        for laserIndex in range(i + mppl_2 + 1, j - mppl_2 - 1):
#            if scan.ranges[laserIndex] > maximum_range:
#                continue
#            distIndex = laserIndex - i - mppl_2
#            if distances[distIndex] >= distances[distIndex - 1] and \
#               distances[distIndex] >= distances[distIndex + 1] and \
#               distances[distIndex] > worstDistance:
#                worstLaserIndex = laserIndex
#                worstDistance = distances[distIndex]
#        if debug:
#            rospy.loginfo( "\tworstLaserIndex: " + str(worstLaserIndex))
#            rospy.loginfo( "\tworstDistance: " + str(worstDistance))
#
#        if worstDistance < orthog_distance_threshold:
#            # We have dealt with this sublist.  Add it to lines.
#            if debug: rospy.loginfo('\tAdding line')
#            lines.lines.append(line)
#        else:
#            # Split this sublist into two and add both back into L.
#            if debug: 
#                rospy.loginfo('\tSplitting: ({}, {})'.format(i, j))
#            insert_sublist_if_long(L, 0, (i, worstLaserIndex-1))
#            insert_sublist_if_long(L, 1, (worstLaserIndex+1, j))
#
#    if debug: rospy.loginfo( "\n\n")
#    
#    # Normally the merge step would follow...
#
#    extracted_publisher.publish(lines)
    rospy.loginfo("STOP scan_callback")
    
def base_scan_callback(msg):
    pdb.set_trace()
    black_board.scan = copy.deepcopy(msg)
    base_scan = list(msg.ranges)
    
    black_board.scan.ranges = []
    for i in range(len(base_scan)):
        rho = base_scan[i]
        if rho == float('inf') or math.isnan(rho):
            black_board.scan.ranges.append(black_board.maximum_range + 1)
        else:
            black_board.scan.ranges.append(rho)
    print base_scan
    print black_board.scan
    rospy.Publisher('/scan', sensor_msgs.msg.LaserScan, queue_size=10).publish(black_board.scan)

def display_callback(lines):
    """Callback for extracted lines, displayed by publishing to
    vis_lines_topic and vis_scanpoints_topic."""
    pdb.set_trace()
    create_lines_marker(lines)
    create_scanpoints_marker(lines)


if __name__ == '__main__':
    pdb.set_trace()
    global extracted_publisher
    global lines_publisher, scanpoints_publisher
 
    # Initialize this node.
    rospy.init_node('wall_follow')

    # Subscribe to /base_scan
    rospy.loginfo("Waiting for /base_scan topic...")
    rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
    rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, base_scan_callback, queue_size=10)
    rospy.sleep(1)
    
    # Subscribe to /scan
    rospy.loginfo("Waiting for /scan topic...")
    rospy.wait_for_message('/scan', sensor_msgs.msg.LaserScan)
    rospy.Subscriber('/scan', sensor_msgs.msg.LaserScan, scan_callback, queue_size=10)
    rospy.sleep(1)
    
    black_board.publisher = rospy.Publisher('/scan', sensor_msgs.msg.LaserScan, queue_size=10)

    # Create our publisher for the lines extracted within 'scan_callback'
    black_board.extracted_publisher = rospy.Publisher('/extracted_lines', ExtractedLines, queue_size=10)
    # Subscribe to 'lines_topic'
    rospy.loginfo("Waiting for lines_topic...")
    rospy.wait_for_message('lines_topic', ExtractedLines)
    rospy.Subscriber(black_board.lines_topic, ExtractedLines, display_callback, queue_size=10)
    rospy.sleep(1)
    # We will publish to vis_lines_topic which will show the lines (as
    # line segments) in RViz.  We will also publish the first and last scan
    # points to topic vis_scanpoints_topic in the same colour so that it
    # is clear from what range of scan points the lines are generated.
    black_board.lines_publisher = rospy.Publisher(black_board.vis_lines_topic, Marker, queue_size=10)
    black_board.scanpoints_publisher = rospy.Publisher(black_board.vis_scanpoints_topic, Marker, queue_size=10)

    rospy.spin()
