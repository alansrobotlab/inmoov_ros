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
from load_config_from_param import load_config_from_param

servos = {}     # servo configuration data for robot
joints = {}     # dict of joint names and position values

bus = {}        # dict of motorcommand busses indexed by ordinal

def init():

    rospy.init_node('joint_command_dispatcher', anonymous=False)
    rate = rospy.Rate(20) # 40hz

    rospy.Subscriber("joint_command", JointState, dispatcher)

    servos = load_config_from_param()

    for j,b in rospy.get_param('/joints').items():
        
        number = rospy.get_param('/joints/' + j + '/bus')
        busname = '/servobus/' + str(number).zfill(2) + '/motorcommand'

        if not bus.has_key(number):
            bus[number] = rospy.Publisher(busname, MotorCommand, queue_size=10)
            rospy.loginfo('adding:  ' + busname)

    while not rospy.is_shutdown():

        #iterate through joints and publish
        for j,p in joints.items():

            try:
                motorcommand = MotorCommand()
                motorcommand.id = int(servos[j].servoPin)
                motorcommand.parameter = PROTOCOL.GOAL
                motorcommand.value = p

                bus[servos[j].bus].publish(motorcommand)
            except:
                rospy.logwarn('joint_command_dispatcher:  unknown joint:' + j)
    
                

        #clear joints cache
        joints.clear()
        rate.sleep()

    rospy.spin()

def dispatcher(js):
    #print "OHAI!"

    # iterate through array and stuff name + position into dict object
    for x in range(0, len(js.name)):
        joints[js.name[x]] = js.position[x]
        print"YATZEE"




if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass