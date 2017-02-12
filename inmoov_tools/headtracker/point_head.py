#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Anh Tran

# Based on the wubble_head_action.py script by Anh Tran. Modifications made by Patrick Goebel
# for the Pi Robot Project.

import roslib; #roslib.load_manifest('pi_head_tracking_3d_part1')
import rospy
import tf
from geometry_msgs.msg import PointStamped
#from std_msgs.msg import Float64
from inmoov_msgs.msg import MotorCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import math

class PointHeadNode():

    def __init__(self):
        # Initialize new node
        rospy.init_node('point_head_node', anonymous=True)
        
        #dynamixel_namespace = rospy.get_namespace()
        rate = rospy.get_param('~rate', 40)
        r = rospy.Rate(rate)
        
        # Initialize the target point
        self.target_point = PointStamped()
        self.last_target_point = PointStamped()
        
        # Subscribe to the target_point topic
        #rospy.Subscriber('/clicked_point', PointStamped, self.update_target_point)
        rospy.Subscriber('/target_point', PointStamped, self.update_target_point)

        # Initialize publisher
        self.head_pan_frame = 'head'
        self.head_tilt_frame = 'head_base'
        self.head_yaw_frame = 'head_tilt'
        self.reference_frame = 'kinect2_link'
        self.head_pub = rospy.Publisher('/servobus/torso/motorcommand', MotorCommand, queue_size=10)
        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)

        
        # Initialize publisher for the tilt servo
        #self.head_tilt_frame = 'head_tilt_link'
        #self.head_tilt_pub = rospy.Publisher(dynamixel_namespace + 'head_tilt_controller/command', Float64)

        # Initialize tf listener
        self.tf = tf.TransformListener()

        self.motorcommand = MotorCommand()
        self.jointcommand = JointState()
        
        # Make sure we can see at least the pan and tilt frames
        #self.tf.waitForTransform(self.head_pan_frame, self.head_tilt_frame, rospy.Time(), rospy.Duration(5.0))
            
        # Reset the head position to neutral
        rospy.sleep(1)
        self.center_head()
        
        rospy.loginfo("Ready to accept target point")
        
        while not rospy.is_shutdown():
            rospy.wait_for_message('/target_point', PointStamped)
            if self.target_point == self.last_target_point:
                continue
            try:
                target_angles = self.transform_target_point(self.target_point)
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.logerr("tf Failure")
                continue
                
            self.head_pan(target_angles[0])
            self.head_tilt(target_angles[1])
            #self.head_pan_pub.publish(target_angles[0])
            #self.head_tilt_pub.publish(target_angles[1])
            
            self.last_target_point = self.target_point
            rospy.loginfo("Setting Target Point:\n" + str(self.target_point))
            
            r.sleep()
        
    def update_target_point(self, msg):
        self.target_point = msg

    def center_head(self):
        self.head_pan(0.0)
        self.head_tilt(0.0)
        #self.head_pan_pub.publish(0.0)
        #self.head_tilt_pub.publish(0.0)
        rospy.sleep(3)

    def transform_target_point(self, target):
        # Set the pan and tilt reference frames to the head_pan_frame and head_tilt_frame defined above
        #pan_ref_frame = self.head_pan_frame
        #tilt_ref_frame = self.head_tilt_frame
        pan_ref_frame = self.reference_frame
        tilt_ref_frame = self.reference_frame
        
        # Wait for tf info (time-out in 5 seconds)
        self.tf.waitForTransform(pan_ref_frame, target.header.frame_id, rospy.Time(), rospy.Duration(5.0))
        self.tf.waitForTransform(tilt_ref_frame, target.header.frame_id, rospy.Time(), rospy.Duration(5.0))

        # Transform target point to pan reference frame & retrieve the pan angle
        pan_target = self.tf.transformPoint(pan_ref_frame, target)
        pan_angle = math.atan2(pan_target.point.y, pan_target.point.x)

        # Transform target point to tilt reference frame & retrieve the tilt angle
        tilt_target = self.tf.transformPoint(tilt_ref_frame, target)
        tilt_angle = math.atan2(tilt_target.point.z,
                math.sqrt(math.pow(tilt_target.point.x, 2) + math.pow(tilt_target.point.y, 2)))

        return [pan_angle, tilt_angle]

    def rad_to_degrees (self, rad):
        degrees = (180.0 * (rad / 3.1415928539))
        rospy.loginfo(str(rad) + " Rad to " + str(degrees) + " Degrees:  \n" )
        
        return (180.0 * (rad / 3.1415928539))

    def tilt_to_tilt_yaw (self, rad):
        return [0,0]

    def head_pan(self, rad):
        self.motorcommand.id = 3
        self.motorcommand.parameter = 0x1E
        self.motorcommand.value = self.rad_to_degrees(rad)
        self.head_pub.publish(self.motorcommand)

        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = ['head_leftright']
        self.jointcommand.position = [rad]
        self.jointcommand.velocity = []
        self.jointcommand.effort = []
        self.jointPublisher.publish(self.jointcommand)

    def head_tilt(self, rad):
        self.motorcommand.id = 4
        self.motorcommand.parameter = 0x1E
        self.motorcommand.value = -self.rad_to_degrees(rad)
        self.head_pub.publish(self.motorcommand)

        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = ['head_updown']
        self.jointcommand.position = [rad]
        self.jointcommand.velocity = []
        self.jointcommand.effort = []
        self.jointPublisher.publish(self.jointcommand)

if __name__ == '__main__':
    try:
        point_head = PointHeadNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

