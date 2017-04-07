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

from constants import PROTOCOL
from servos import Servo

servos = {}     # servo configuration data for robot
joints = {}     # dict of joint names and position values

bus = {}        # dict of motorcommand busses indexed by ordinal

PI = 3.1415926539

# https://github.com/ColinDuquesnoy/QDarkStyleSheet
import qdarkstyle

# https://www.safaribooksonline.com/blog/2014/01/22/create-basic-gui-using-pyqt/
gui = os.path.join(os.path.dirname(__file__), 'enable_manager.ui')
form_class = uic.loadUiType(gui)[0]

def commandDispatcher(data):

    # if robot set up with degrees, then convert to radians
    if rospy.get_param('/bringup/angles') == 'degrees':
        p = list(data.position)
        for x in range(0, len(p)):
            p[x] = PI * p[x] / 180.0

        data.position = tuple(p)

    publisher.publish(data)

def stateDispatcher(data):

    # if robot set up with degrees, then convert to radians
    if rospy.get_param('/bringup/angles') == 'degrees':
        p = list(data.position)
        for x in range(0, len(p)):
            p[x] = PI * p[x] / 180.0

        data.position = tuple(p)

    publisher.publish(data)

subscriber1 = rospy.Subscriber("joint_command", JointState, commandDispatcher)
subscriber2 = rospy.Subscriber("joint_state", JointState, stateDispatcher)
publisher = rospy.Publisher('rviz_command',JointState, queue_size=10)

def init():

    rospy.init_node('rviz_command_dispatcher', anonymous=False)
    #rate = rospy.Rate(20) # 20hz

    load_config_from_param()

    #for j,b in rospy.get_param('/joints').items():
    #    
    #    number = rospy.get_param('/joints/' + j + '/bus')
    #    busname = '/servobus/' + str(number).zfill(2) + '/motorcommand'

    #    if not bus.has_key(number):
    #        bus[number] = rospy.Publisher(busname, MotorCommand, queue_size=10)
    #        rospy.loginfo('adding:  ' + busname)


    rospy.spin()

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

        s.bus       =  int(rospy.get_param(key + 'bus'))
        s.servo     =  int(rospy.get_param(key + 'servo'))
        s.flip      =  int(rospy.get_param(key + 'flip'))

        s.servopin  =  int(rospy.get_param(key + 'servopin'))
        s.sensorpin =  int(rospy.get_param(key + 'sensorpin'))
        s.minpulse  =  int(rospy.get_param(key + 'minpulse'))
        s.maxpulse  =  int(rospy.get_param(key + 'maxpulse'))
        s.minangle  =  float(rospy.get_param(key + 'minangle'))
        s.maxangle  =  float(rospy.get_param(key + 'maxangle'))
        s.minsensor =  int(rospy.get_param(key + 'minsensor'))
        s.maxsensor =  int(rospy.get_param(key + 'maxsensor'))
        s.maxspeed  =  float(rospy.get_param(key + 'maxspeed'))
        s.smoothing =  int(rospy.get_param(key + 'smoothing'))

        servos[name] = s

    print "DONE"



if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass