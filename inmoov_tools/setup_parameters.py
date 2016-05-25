#!/usr/bin/env python

import sys
import os
import rospy
import rospkg

leftArmPort  = "/dev/ttyACM0"
rightArmPort = "/dev/ttyACM1"
torsoPort    = "/dev/ttyACM2"

rospy.set_param( '/servobus/torso/port', torsoPort)
rospy.set_param( '/servobus/torso/name', 'servoBus/torso')

rospy.set_param( '/servobus/torso/servomap/0/name', 'eye_leftright')
rospy.set_param( '/servobus/torso/servomap/1/name', 'eyes_updown')
rospy.set_param( '/servobus/torso/servomap/2/name', 'jaw')
rospy.set_param( '/servobus/torso/servomap/3/name', 'head_leftright')
rospy.set_param( '/servobus/torso/servomap/4/name', 'head_updown')
rospy.set_param( '/servobus/torso/servomap/5/name', 'head_tilt')
rospy.set_param( '/servobus/torso/servomap/6/name', 'waist_lean')
rospy.set_param( '/servobus/torso/servomap/7/name', 'waist_rotate')
rospy.set_param( '/servobus/torso/servomap/8/name', 'torso-nc-8')
rospy.set_param( '/servobus/torso/servomap/9/name', 'torso-nc-9')
rospy.set_param( '/servobus/torso/servomap/10/name', 'torso-nc-10')
rospy.set_param( '/servobus/torso/servomap/11/name', 'torso-nc-11')

rospy.set_param('/servobus/leftarm/port', leftArmPort)
rospy.set_param('/servobus/leftarm/name', 'servoBus/leftarm')

rospy.set_param( '/servobus/leftarm/servomap/0/name', 'left_pinky')
rospy.set_param( '/servobus/leftarm/servomap/1/name', 'left_ring')
rospy.set_param( '/servobus/leftarm/servomap/2/name', 'left_middle')
rospy.set_param( '/servobus/leftarm/servomap/3/name', 'left_index')
rospy.set_param( '/servobus/leftarm/servomap/4/name', 'left_thumb')
rospy.set_param( '/servobus/leftarm/servomap/5/name', 'left_hand')
rospy.set_param( '/servobus/leftarm/servomap/6/name', 'left_bicep')
rospy.set_param( '/servobus/leftarm/servomap/7/name', 'left_bicep_rotate')
rospy.set_param( '/servobus/leftarm/servomap/8/name', 'left_shoulder_side')
rospy.set_param( '/servobus/leftarm/servomap/9/name', 'left_shoulder_up')
rospy.set_param( '/servobus/leftarm/servomap/10/name', 'leftarm-nc-10')
rospy.set_param( '/servobus/leftarm/servomap/11/name', 'leftarm-nc-11')

rospy.set_param('/servobus/rightarm/port', rightArmPort)
rospy.set_param('/servobus/rightarm/name', 'servoBus/rightarm')

rospy.set_param( '/servobus/rightarm/servomap/0/name', 'right_pinky')
rospy.set_param( '/servobus/rightarm/servomap/1/name', 'right_ring')
rospy.set_param( '/servobus/rightarm/servomap/2/name', 'right_middle')
rospy.set_param( '/servobus/rightarm/servomap/3/name', 'right_index')
rospy.set_param( '/servobus/rightarm/servomap/4/name', 'right_thumb')
rospy.set_param( '/servobus/rightarm/servomap/5/name', 'right_hand')
rospy.set_param( '/servobus/rightarm/servomap/6/name', 'right_bicep')
rospy.set_param( '/servobus/rightarm/servomap/7/name', 'right_bicep_rotate')
rospy.set_param( '/servobus/rightarm/servomap/8/name', 'right_shoulder_side')
rospy.set_param( '/servobus/rightarm/servomap/9/name', 'right_shoulder_up')
rospy.set_param( '/servobus/rightarm/servomap/10/name', 'rightarm-nc-10')
rospy.set_param( '/servobus/rightarm/servomap/11/name', 'rightarm-nc-11')

print("Parameter Server Setup for Torso:")
print(rospy.get_param("/servobus/torso"))
print("")

print("Parameter Server Setup for LeftArm:")
print(rospy.get_param("/servobus/leftarm"))
print("")

print("Parameter Server Setup for RightArm:")
print(rospy.get_param("/servobus/rightarm"))
print("")