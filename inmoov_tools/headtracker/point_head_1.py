#! /usr/bin/env python

# Based on the wubble_head_action.py script by Anh Tran. Modifications made by Patrick Goebel
# for the Pi Robot Project.


import roslib;
import rospy
import tf
from geometry_msgs.msg import PointStamped
from inmoov_msgs.msg import MotorCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import math
import time

class PointHeadNode():

    def __init__(self):
        # Initialize new node
        rospy.init_node('point_head_node', anonymous=True)
        
        self.rate = 0.2
        self.headMoveDuration = 1000.0  #time in milliseconds

        rate = rospy.get_param('~rate', self.rate)
        r = rospy.Rate(rate)

        # Subscribe to the target_point topic
        rospy.Subscriber('/target_point', PointStamped, self.update_target_point)

        # Initialize publisher
        self.head_pan_frame = 'head'
        self.head_tilt_frame = 'head'
        self.head_yaw_frame = 'head_tilt'
        self.reference_frame = 'kinect2_link'
        self.head_pub = rospy.Publisher('/servobus/torso/motorcommand', MotorCommand, queue_size=10)
        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)

        # Initialize the target point
        self.target_point = PointStamped()
        self.target_point.point.x = 100.0
        self.target_point.point.y = 0.0
        self.target_point.point.z = 0.0
        self.target_point.header.stamp = rospy.Time.now()
        self.target_point.header.frame_id = self.reference_frame

        self.last_target_point = PointStamped()

        # Initialize tf listener
        self.tf = tf.TransformListener()

        self.motorcommand = MotorCommand()
        self.jointcommand = JointState()

        self.center_head()
        
        # Make sure we can see at least the pan and tilt frames
        self.tf.waitForTransform(self.head_pan_frame, self.head_tilt_frame, rospy.Time(), rospy.Duration(5.0))
            
        rospy.sleep(1)
        
        rospy.loginfo("Ready to accept target point")

        self.center_head()
        
        while not rospy.is_shutdown():
            
            r.sleep()
        
    def update_target_point(self, msg):

        try:
            target_angles = self.transform_target_point(msg)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("tf Failure")

        self.head_pan(target_angles[0])
        self.head_tilt(target_angles[1])



    def center_head(self):
        self.head_pan(0.0)
        self.head_tilt(0.0)
        rospy.sleep(3)

    def transform_target_point(self, target):
        # Set the pan and tilt reference frames to the head_pan_frame and head_tilt_frame defined above
        ref_frame = 'base_link'

        target.point.z -= 1.35

        rospy.loginfo("OG Point=" + str(target))

        target.header.stamp = rospy.Time(0)
        
        # Wait for tf info (time-out in 5 seconds)
        self.tf.waitForTransform(ref_frame, target.header.frame_id, rospy.Time(), rospy.Duration(5.0))

        # Transform target point to pan reference frame & retrieve the pan angle
        pan_target = self.tf.transformPoint(ref_frame, target)
        pan_angle = math.atan2(pan_target.point.y, pan_target.point.x)

        # Transform target point to tilt reference frame & retrieve the tilt angle
        tilt_target = self.tf.transformPoint(ref_frame, target)
        tilt_angle = -math.atan2(tilt_target.point.z,
                math.sqrt(math.pow(tilt_target.point.x, 2) + math.pow(tilt_target.point.y, 2)))

        rospy.loginfo("New Point=" + str(pan_target))

        rospy.loginfo(":Pan Angle=" + str(self.rad_to_degrees(pan_angle)) + " Tilt Angle=" + str(self.rad_to_degrees(tilt_angle)))

        return [pan_angle, tilt_angle]

    def rad_to_degrees (self, rad):
        degrees = (180.0 * (rad / 3.1415928539))
        #rospy.loginfo(str(rad) + " Rad to " + str(degrees) + " Degrees:  \n" )
        
        return (180.0 * (rad / 3.1415928539))

    def tilt_to_tilt_yaw (self, rad):
        return [0,0]

    def nudge(self):

        m = self.getMillis()-self.startMillis

        self.curPanAngle = self.targetPanAngle
        self.curTiltAngle = self.targetTiltAngle

        self.head_pan(self.curPanAngle)
        self.head_tilt(self.curTiltAngle)


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
        self.motorcommand.value = self.rad_to_degrees(rad)
        self.head_pub.publish(self.motorcommand)

        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = ['head_updown']
        self.jointcommand.position = [rad]
        self.jointcommand.velocity = []
        self.jointcommand.effort = []
        self.jointPublisher.publish(self.jointcommand)

    def head_roll(self, rad):
        self.motorcommand.id = 5
        self.motorcommand.parameter = 0x1E
        self.motorcommand.value = self.rad_to_degrees(rad)
        self.head_pub.publish(self.motorcommand)

        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = ['head_tilt']
        self.jointcommand.position = [rad]
        self.jointcommand.velocity = []
        self.jointcommand.effort = []
        self.jointPublisher.publish(self.jointcommand)

    def eyes_pan(self, rad):
        self.motorcommand.id = 0
        self.motorcommand.parameter = 0x1E
        self.motorcommand.value = self.rad_to_degrees(rad)
        self.head_pub.publish(self.motorcommand)

        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = ['eye_leftright']
        self.jointcommand.position = [rad]
        self.jointcommand.velocity = []
        self.jointcommand.effort = []
        self.jointPublisher.publish(self.jointcommand)

    def eyes_tilt(self, rad):
        self.motorcommand.id = 1
        self.motorcommand.parameter = 0x1E
        self.motorcommand.value = self.rad_to_degrees(rad)
        self.head_pub.publish(self.motorcommand)

        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = ['eyes_updown']
        self.jointcommand.position = [rad]
        self.jointcommand.velocity = []
        self.jointcommand.effort = []
        self.jointPublisher.publish(self.jointcommand)

    def getMillis(self):
        #return time.time()*1000
        m = int(round(time.time()*1000))
        return m

if __name__ == '__main__':
    try:
        point_head = PointHeadNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

