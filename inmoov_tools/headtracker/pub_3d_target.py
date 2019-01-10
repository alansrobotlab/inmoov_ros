#!/usr/bin/env python

#"""
#    pub_3d_target.py - Version 1.0 2011-08-17
#    
##    Publish a target point relative to a given reference frame.
#  
#   Created for the Pi Robot Project: http://www.pirobot.org
#    Copyright (c) 2010 Patrick Goebel.  All rights reserved.
#
#   This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#    
#    This program is distributed in the hope that it will be useful,
##    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details at:
#    
#    http://www.gnu.org/licenses/gpl.html
#"""

import roslib; #roslib.load_manifest('pi_head_tracking_3d_part2')
import rospy
import tf
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
import math
import random

class Pub3DTarget():
    def __init__(self):
        rospy.init_node('pub_3d_target')
                
        rate = rospy.get_param('~rate', 40)
        r = rospy.Rate(rate)
        
        # Remap this frame in the launch file or command line if necessary
        target_frame = 'kinect2_link'
        target_topic = 'target_point'
        #target_topic = 'clicked_point'

        # Parameters for publishing the RViz target marker
        marker_scale = rospy.get_param('~marker_scale', 0.05)
        marker_lifetime = rospy.get_param('~marker_lifetime', 1) # 0 is forever
        marker_ns = rospy.get_param('~marker_ns', 'target_point')
        marker_id = rospy.get_param('~marker_id', 0)
        marker_color = rospy.get_param('~marker_color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})
        
        # Define a marker publisher
        marker_pub = rospy.Publisher('target_point_marker', Marker)
        
        # Initialize the marker
        marker = Marker()
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(marker_lifetime)
        marker.scale.x = marker_scale
        marker.scale.y = marker_scale
        marker.scale.z = marker_scale
        marker.color.r = marker_color['r']
        marker.color.g = marker_color['g']
        marker.color.b = marker_color['b']
        marker.color.a = marker_color['a']
        
        # Define the target as a PointStamped message
        target = PointStamped()
        target.header.frame_id = target_frame
        
        # Define the target publisher
        target_pub = rospy.Publisher(target_topic, PointStamped)

        # Initialize tf listener       
        tf_listener = tf.TransformListener()
        
        # Give the tf buffer a chance to fill up
        rospy.sleep(5.0)
        
        rospy.loginfo("Publishing target point on frame " + str(target_frame))
        
        theta = 0.0
        rand_num = random.Random()
        
        while not rospy.is_shutdown():
            target.point.x = 0.5 + 0.25 * math.sin(theta)
            target.point.y = 0.5 * math.sin(theta)
            target.point.z = -0.1 + 0.5 * abs(math.sin(theta))
            theta += 0.025
            target.header.stamp = rospy.Time.now()
            target_pub.publish(target)
            
            marker.header.stamp = target.header.stamp
            marker.header.frame_id = 'base_link'
            marker.pose.position = target.point
            marker.id += 1
            marker.id %= 100
            marker_pub.publish(marker)                       

            r.sleep()
                   
if __name__ == '__main__':
    try:
        target = Pub3DTarget()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target publisher is shut down.")
