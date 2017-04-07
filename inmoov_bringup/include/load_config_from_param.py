#!/usr/bin/env python
# licensed under BSD-3

import sys
import rospy
import yaml
import os
from os.path import dirname, abspath


#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(abspath(__file__))),'include'))

from constants import PROTOCOL
from servos import Servo

servos = {}

def load_config_from_param():

    # first, make sure parameter server is even loaded
    while not rospy.search_param("/joints"):
        rospy.loginfo("waiting for parameter server to load with joint definitions")
        rospy.sleep(1)

    rospy.sleep(1)

    joints = rospy.get_param('/joints')
    for name in joints:
        rospy.loginfo( "found:  " + name )

        s = Servo()

        key = '/joints/' + name + '/'

        s.bus       =  rospy.get_param(key + 'bus')
        s.servo     =  rospy.get_param(key + 'servo')

        s.servoPin  =  rospy.get_param(key + 'servoPin')
        s.minPulse  =  rospy.get_param(key + 'minPulse')
        s.maxPulse  =  rospy.get_param(key + 'maxPulse')
        s.minGoal   =  rospy.get_param(key + 'minGoal')
        s.maxGoal   =  rospy.get_param(key + 'maxGoal')
        s.rest      =  rospy.get_param(key + 'rest')
        s.maxSpeed  =  rospy.get_param(key + 'maxSpeed')
        s.smoothing =  rospy.get_param(key + 'smoothing')

        s.sensorpin =  rospy.get_param(key + 'sensorPin')
        s.minSensor =  rospy.get_param(key + 'minSensor')
        s.maxSensor =  rospy.get_param(key + 'maxSensor')

        servos[name] = s

    return servos
