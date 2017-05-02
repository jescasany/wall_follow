#!/usr/bin/env python 
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 27 12:50:35 2017

Subscribes to the /base_scan topic and publishes a filtered
laser scan on black_board.scan variable.  Unusable values such as 'inf' and 'nan' in /base_scan
are converted to maximum_range + 1 in black_board.scan.

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
    - Incorporated laser filter which eliminates data points with infinite
      or invalid range values.
    - Using a modifed version of the split-and-merge algorithm to better handle
      the presence of multiple walls by selecting the worst-fit point to be a
      local maximum in orthogonal line distance.
    - Added a constraint on the line fitting procedure to prevent negative
      r values.

Extracted lines subscribes to the /base_scan topic and applies the
split-and-merge algorithm to fit multiple lines to this data.  These lines are
published on the /extracted_lines topic.

Display Lines displays lines extracted from laser range scans.  The node
subscribes to 'lines_topic' and publishes the data needed to display these
lines to 'vis_lines_topic' and correspondingly coloured first and last scan points used to generate each line to 'vis_scanpoints_topic'.  RViz must be
configured appropriately to display the messages posted to these topics.

@author: juan
"""

import pdb

import rospy
import math

import sensor_msgs.msg
from comp4766_a3_mod.msg import ExtractedLine
from comp4766_a3_mod.msg import ExtractedLines

from geometry_msgs.msg import Point32, Twist, Point
from angles import constrain_angle
from pi_trees_ros.pi_trees_ros import *

from visualization_msgs.msg import Marker
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
        
        self.scan = list()
        
        # Create our publisher for the lines extracted within 'base_scan' callback
        self.extracted_publisher = rospy.Publisher(self.lines_topic, ExtractedLines, queue_size=10)
        
        # We will publish to vis_lines_topic which will show the lines (as
        # line segments) in RViz.  We will also publish the first and last scan
        # points to topic vis_scanpoints_topic in the same colour so that it
        # is clear from what range of scan points the lines are generated.
        self.lines_publisher = rospy.Publisher(self.vis_lines_topic, Marker, queue_size=10)
        self.scanpoints_publisher = rospy.Publisher(self.vis_scanpoints_topic, Marker, queue_size=10)
        
        
black_board = BlackBoard()

# The min_points_per_line parameter should be an even number greater than 0.
assert black_board.min_points_per_line % 2 == 0
assert black_board.min_points_per_line > 0


class WallFollow:
    def __init__(self):
        #pdb.set_trace()
        rospy.init_node("wall_follow")
        # Set the shutdown function (stop the agent)
        rospy.on_shutdown(self.shutdown)
        # Publisher to manually control the agent (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
         # Create the root node
        WALLFOLLOW = Sequence("WALLFOLLOW")
        
        START_STEP = CallbackTask("START STEP", self.step)
        
        I_F_IS_VISITED =IgnoreFailure("I_F IS VISITED")
        
        IS_VISITED = CallbackTask("is visited", self.is_visited)
        
        WALLFOLLOW.add_child(START_STEP)
        WALLFOLLOW.add_child(I_F_IS_VISITED)
        
        # Display the tree before beginning execution
        print_tree(WALLFOLLOW, indent=0, use_symbols=True)
        print_dot_tree(WALLFOLLOW, dotfilepath='/home/juan/catkin_ws/src/adaptor001/tree02.dot')
        
        # Run the tree
        while not rospy.is_shutdown():
            #pdb.set_trace()
            WALLFOLLOW.run()
            
    def formule(self, L, tolerance):
        f = 0
        f = L[1] - (L[0] + L[2])/2
        f = self.round_to_zero(f, tolerance)
        return f
        
    def moving_window_filtro(self, x, tolerance=0.2, n_neighbors=1):
        n = len(x)
        width = n_neighbors*2 + 1
        x = [x[0]]*n_neighbors + x + [x[-1]]*n_neighbors
        # To complete the function,
        # return a list of the filtered values from i to i+width for all values i from 0 to n-1.
        filtro = []
        singularity = [0]
        last_sing = n-1
        for i in range(n):
            fi = abs(self.formule(x[i:i+width], tolerance))
            filtro.append(fi)
            if fi != 0.0:
                if i != last_sing + 1:
                    singularity.append(i)
                    last_sing = i
        singularity.append(n-1)
            
        return filtro, singularity
    
    def round_to_zero(self, val, tolerance):
        if -1 * tolerance < val < tolerance:
            return 0
        else:
            return val
            
    def step(self):
        pdb.set_trace()
        self.laser_scan()
        """
        Extracts lines from the given LaserScan and publishes to /extracted_lines.
    
        Extracts lines using the split phase of the split-and-merge algorithm.
        Publish these lines as an ExtractedLines method on the /extracted_lines
        topic.
        """
        # Create an ExtractedLines object and initialize some fields in the header.
        lines = ExtractedLines()
        lines.header.frame_id = '/odom'
        #lines.header.frame_id = '/base_laser_link'
        lines.header.stamp = rospy.Time.now()
        # Create our big list of index pairs.  Each pair gives the start and end
        # index which specifies a contiguous set of data points in 'scan'.
        n = len(black_board.scan)
        end_index = n-1
        done_si = False
        for i in range(n):
            if black_board.scan[i] < black_board.maximum_range and black_board.scan[i] != 0.0:
                if not done_si:
                    start_index = i
                    done_si = True
                end_index = i
        line = self.fit_line(black_board.scan, start_index, end_index, black_board.maximum_range)
    
        if line is not None:
            lines.lines.append(line)
            print lines
        # Normally the merge step would follow...
        
        #raw_input('Enter to continue')
        
        # Subscribe to 'lines_topic'
        rospy.Publisher('/extracted_lines', ExtractedLines, queue_size=10).publish(lines)
        #rospy.loginfo("Waiting for lines_topic...")
        #rospy.wait_for_message('/extracted_lines', ExtractedLines)
        rospy.Subscriber('/extracted_lines', ExtractedLines, self.display_callback, queue_size=10)
            
        rospy.sleep(1)
        
    def is_visited(self):
        return 1
        
    def fit_line(self, scan, start_index, end_index, maximum_range):
        
        # First we must calculate alpha, using the formula presented in the
        # course notes (due to Arras, 1998).
        sumWeights = 0
        sumNumL = 0 # The summation term in the numerator on the left
        sumNumR = 0 # The summation term in the numerator on the right
        sumDenL = 0 # The summation term in the denominator on the left
        sumDenR = 0 # The summation term in the denominator on the rt.
        for i in range(start_index, end_index+1):  
            rho = scan[i]
            if rho == 0 or rho > maximum_range:
                continue
            theta = black_board.angle_min + i * black_board.angle_increment
            weight = 1 / rho**2
            #weight = 1
            sumWeights += weight
    
            factor1 = weight * rho * rho
            sumNumL += factor1 * math.sin(2 * theta)
            sumDenL += factor1 * math.cos(2 * theta)
            
            for j in range(start_index, end_index+1):
                rho_j = scan[j]
                if rho_j == 0 or rho_j > maximum_range:
                    continue
                theta_j = black_board.angle_min + j * black_board.angle_increment
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
            rho = scan[i]
            if rho == 0 or rho > maximum_range:
                continue
            theta = black_board.angle_min + i * black_board.angle_increment
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
        #pdb.set_trace()
        # for visualization to show the range of points involved.
        firstScanPoint = Point32()
        lastScanPoint = Point32()
        dist = scan[start_index]
        angle = black_board.angle_min + start_index * black_board.angle_increment
        if dist <= maximum_range:
            firstScanPoint.x = dist * math.cos(angle)
            firstScanPoint.y = dist * math.sin(angle)
        dist = scan[end_index]
        angle = black_board.angle_min + end_index * black_board.angle_increment
        if dist <= maximum_range:
            lastScanPoint.x = dist * math.cos(angle)
            lastScanPoint.y = dist * math.sin(angle)
        
        return ExtractedLine(r, alpha, firstScanPoint, lastScanPoint)
        
    def create_lines_marker(self, lines):
        pdb.set_trace()
        marker_lines = Marker()
    
        # Get the pairs of points that comprise the endpoints for all lines
        marker_lines.points = self.generate_endpoints(lines)
    
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
    
        black_board.lines_publisher.publish(marker_lines)
    
    def create_scanpoints_marker(self, lines):
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

    def generate_endpoints(self, lines):
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
            #print points
        return points

    def display_callback(self, lines):
        """Callback for extracted lines, displayed by publishing to
        vis_lines_topic and vis_scanpoints_topic."""
        #pdb.set_trace()
        self.create_lines_marker(lines)
        self.create_scanpoints_marker(lines)
    
    def laser_scan(self):
        rospy.loginfo("Waiting for /base_scan topic...")
        rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
        # Subscribe the /base_scan topic to get the range readings  
        rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, self.scan_callback, queue_size = 10)
        rospy.sleep(0.1)

        for i in range(len(black_board.scan)):
            rho = black_board.scan[i]
            if rho == float('inf') or math.isnan(rho):
                black_board.scan[i] = (black_board.maximum_range + 1)    
        
        rospy.loginfo("laser_scan done")
        
    def scan_callback(self, msg):
        black_board.scan = list(msg.ranges) # transformed to list since msg.ranges is a tuple
        black_board.angle_min = msg.angle_min
        black_board.angle_increment = msg.angle_increment
        
    def shutdown(self):
        rospy.loginfo("Stopping the agent...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    
    tree = WallFollow()
    
