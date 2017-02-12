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
                
        rate = rospy.get_param('~rate', 10)
        r = rospy.Rate(rate)
        
        # Remap this frame in the launch file or command line if necessary
        #target_frame = 'base_link'
        #target_frame = 'kinect2_link'
        target_topic = 'target_point'
        #target_topic = 'clicked_point'

        # Parameters for publishing the RViz target marker
        marker_scale = rospy.get_param('~marker_scale', 0.1)
        marker_lifetime = rospy.get_param('~marker_lifetime', 1) # 0 is forever
        marker_ns = rospy.get_param('~marker_ns', 'target_point')
        marker_id = rospy.get_param('~marker_id', 0)
        marker_color = rospy.get_param('~marker_color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})
        
        # Define a marker publisher
        self.marker_pub = rospy.Publisher('target_point_marker', Marker, queue_size=10)

        self.clicked_sub = rospy.Subscriber('clicked_point', PointStamped, self.clicked_callback)

        #target_pub = rospy.Publisher(target_topic, PointStamped, queue_size=10)
        
        # Initialize the marker
        self.marker = Marker()
        self.marker.ns = marker_ns
        self.marker.id = marker_id
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.lifetime = rospy.Duration(marker_lifetime)
        self.marker.scale.x = marker_scale
        self.marker.scale.y = marker_scale
        self.marker.scale.z = marker_scale
        self.marker.color.r = marker_color['r']
        self.marker.color.g = marker_color['g']
        self.marker.color.b = marker_color['b']
        self.marker.color.a = marker_color['a']
        
        # Define the target as a PointStamped message
        self.target = PointStamped()
        self.target.point.x = 100.0
        self.target.point.y = 0.0
        self.target.point.z = 0.0
        self.target.header.stamp = rospy.Time.now()
        self.target.header.frame_id = 'base_link'

        # Define the target publisher
        self.target_pub = rospy.Publisher(target_topic, PointStamped, queue_size=10)

        # Initialize tf listener       
        self.tf = tf.TransformListener()
        
        # Give the tf buffer a chance to fill up
        rospy.sleep(5.0)
        
        rospy.loginfo("Publishing target point on frame ")# + str(target_frame))
        
        while not rospy.is_shutdown():                   

            r.sleep()


    def clicked_callback(self, data):
        #i dont know why yet, but clicked_point has x and y reversed

        #self.target.point = self.tf.transformPoint('kinect2_link', data)
        #self.target.point.x = data.point.x
        #self.target.point.y = data.point.y
        #self.target.point.z = data.point.z
        #self.target.header.frame_id = data.header.frame_id
        
        #self.target.header.stamp = rospy.Time.now()
        self.target = data
        self.target_pub.publish(self.target)

        self.marker.header.stamp = self.target.header.stamp
        self.marker.header.frame_id = self.target.header.frame_id
        self.marker.pose.position = self.target.point
        self.marker.id += 1
        self.marker.id %= 100
        self.marker_pub.publish(self.marker)   


if __name__ == '__main__':
    try:
        target = Pub3DTarget()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target publisher is shut down.")
