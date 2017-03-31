#!/usr/bin/env python
# licensed under BSD-3


import rospy

from inmoov_msgs.msg import MotorStatus
from inmoov_msgs.msg import MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import os
import sys
from os.path import dirname, abspath

#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(abspath(__file__))),'include'))

from constants import Protocol
from servos import Servo

servos = {}
joints = {}

def init():

    load_config_from_param()
    
    rospy.init_node('joint_command_dispatcher', anonymous=False)
    rate = rospy.Rate(40) # 40hz

    rospy.Subscriber("joint_command", JointState, dispatcher)

    while not rospy.is_shutdown():
        joints
        #iterate through joints and publish

        #clear joints cache
        joints.clear()
        rate.sleep()

    rospy.spin()

def dispatcher(data):
    print "OHAI!"

    js = JointState(date)

    for x in range(0, js.name.count):
        joints[js.name[x]] = js.position[x]
        #servos[js.name[x]].goal = js.position[x]


def load_config_from_param():

    # first, make sure parameter server is even loaded
    while not rospy.search_param("/joints/angles"):
        rospy.loginfo("waiting for parameter server to load with joint definitions")
        rospy.sleep(1)

    rospy.sleep(1)

    i = 0

    while rospy.search_param('/joints/joint' + str(i).zfill(2)):

        param = '/joints/joint' + str(i).zfill(2)

        print "found " + str(i).zfill(2)

        servo = Servo()
        servo.key       =  i
        servo.name      =  rospy.get_param(param + '/name')
        servo.bus       =  rospy.get_param(param + '/bus')
        servo.servo     =  rospy.get_param(param + '/servo')
        servo.flip      =  rospy.get_param(param + '/flip')

        servo.servopin  =  rospy.get_param(param + '/servopin')
        servo.sensorpin =  rospy.get_param(param + '/sensorpin')
        servo.minpulse  =  rospy.get_param(param + '/minpulse')
        servo.maxpulse  =  rospy.get_param(param + '/maxpulse')
        servo.minangle  =  rospy.get_param(param + '/minangle')
        servo.maxangle  =  rospy.get_param(param + '/maxangle')
        servo.minsensor =  rospy.get_param(param + '/minsensor')
        servo.maxsensor =  rospy.get_param(param + '/maxsensor')
        servo.maxspeed  =  rospy.get_param(param + '/maxspeed')
        servo.smoothing =  rospy.get_param(param + '/smoothing')

        servos[servo.name] = servo

        i+=1

    #print servos

    print "DONE"


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass