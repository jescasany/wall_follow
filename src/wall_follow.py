#!/usr/bin/env python  
"""
ROS Node to implement wall following.

The algorithm is based on line extraction. The lines should have already been
published on topic 'extracted_lines'. The overall idea is to follow the wall
on the robot's right. We will follow that wall by setting a goal position that
is pushed forward along the wall by parameter 'follow-advance' and offset
inwards from the wall by distance 'follow-offset'. As soon as a wall in front
of the robot becomes closer, the algorithm will switch to following that wall
using the same strategy.  In either case, we simply react to the closest wall
as long as it lies within a certain angular range which prevents the robot from
reacting to walls to the left. Thus there is no need for any explicit logic to
detect corners or transition from one wall to the other.

Andrew Vardy
"""
import pdb
#import roslib
#roslib.load_manifest('comp4766_a4_p1')
import rospy
from math import pi, cos, sin
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from comp4766_a3_mod.msg import ExtractedLine
from comp4766_a3_mod.msg import ExtractedLines
from smooth1 import SmoothController1

# STUDENTS: LOOK FOR CODE SEGMENTS LABELLED "???" AND REPLACE THEM AS NEEDED.
def lines_callback(lines_msg):
    # The first thing to do is account for the fact that the lines are
    # extracted in the 'laser' reference frame.  The laser scanner is installed
    # upside-down.  We could use tf to account for the difference, but all that
    # we need to adjust is to negate all of the alpha angles for each extracted
    # line.  The translation of the laser from the centre of the robot
    # shouldn't have an impact on the algorithm here other than changing the
    # interpretation of the 'follow-offset' parameter.
    pdb.set_trace()
    for l in lines_msg.lines:
        l.alpha *= -1
    # Set 'line' to be the closest line to the robot.  If the closest line is
    # not ahead of the robot or on it's right side then the 'line' variable will
    # be reset to 'None' indicating there is no suitable line to follow.  We
    # choose a range of alpha values to represent such lines.  
    line = None
    smallestR = float('inf')
    for l in lines_msg.lines:
        if l.r < smallestR:
            smallestR = l.r
            line = l
    # If this closest line is in the right angular range, we will use it below
    # to generate a goal position.  Otherwise, we will ignore it and keep with
    # any previously set velocity.
    if abs(line.alpha) > pi/2 :
        line = None
    # Place the closest line into a new ExtractedLines message and publish it on
    # topic 'selected_lines'.  This will allow us to see the line selected
    # above in rviz.  Note that we create a new line and negate the alpha back
    # to its original value.  This is because rviz will display the line w.r.t.
    # the 'laser' frame.
    sel_lines = ExtractedLines()
    sel_lines.header.frame_id = lines_msg.header.frame_id
    if line != None:
        sel_line = ExtractedLine()
        sel_line.r = line.r
        sel_line.alpha = -line.alpha
        sel_lines.lines.append(sel_line)
    selected_lines_publisher.publish(sel_lines)
    # The position of the goal in the robot reference frame is specified by
    # the values goalx and goaly, the x and y coordinates of the goal in the
    # robot reference frame.  These are obtained by summing the following two
    # vectors which the instructor will illustrate on the board.  Note that fo
    # = follow-offset and fa = follow-advance):
    #
    #   Vector from the origin to the closest point on the line, with a length
    #   of r (orthogonal distance of the line) - fo
    #       [(r-fo) cos(alpha), (r-fo) sin(alpha)]
    #
    #   Vector along the line, oriented towards counter-clockwise with length fa
    #       [fa cos(alpha+pi/2), fa sin(alpha+pi/2)]
    #
    goalx = 0
    goaly = 0
    if line != None:
         fo = follow_offset
         fa = follow_advance
         goalx = (line.r - fo) * cos(line.alpha) + fa * cos(line.alpha + pi/2)
         goaly = (line.r - fo) * sin(line.alpha) + fa * sin(line.alpha + pi/2)

    # Publish the goal's position for visualization
    goalPoint = PointStamped()
    goalPoint.header.frame_id = '/base_link'
    goalPoint.point.x = goalx
    goalPoint.point.y = goaly
    goal_publisher.publish(goalPoint)
    # Set the goal position in the controller and allow it to generate an
    # appropriate Twist message.
    #
    # Comment the following 2 lines when you want your robot to actually move:
    goalx = 0
    goaly = 0
    controller.set_goal(goalx, goaly)
    twist = controller.get_twist()
    # Publish the twist message produced by the controller.
    if line != None:
        cmd_vel_publisher.publish(twist)

if __name__ == '__main__':
    pdb.set_trace()
    global follow_offset, follow_advance, controller

    rospy.init_node('wall_follow')
    # Read the following parameters managed by the ROS parameter server:
    #   follow-advance:  Desired distance to advance along the line
    #   follow-offset: Desired offset from line
    
#    follow_offset = rospy.get_param('~follow_offset')
#    follow_advance = rospy.get_param('~follow_advance')
    follow_offset = 1.5
    follow_advance = 1.0
    # Subscribe to /extracted_lines
    rospy.Subscriber('/extracted_lines', ExtractedLines, lines_callback)

    # Publish to /extracted_lines_wf to visualize the line currently used for
    # wall following.
    selected_lines_publisher = rospy.Publisher('/extracted_lines_wf', \
                               ExtractedLines, queue_size=10)

    # Publish to /goal to visualize the current goal position
    goal_publisher = rospy.Publisher('/goal', PointStamped, queue_size=5)

    # Create a publisher so that we can output command velocities.
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create our instance of smooth controller 1, adapted from assignment 4
    controller = SmoothController1()

    rospy.spin()
