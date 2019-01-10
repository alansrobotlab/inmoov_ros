#!/usr/bin/env python
# licensed under BSD-3


import rospy

from random import randint

from inmoov_msgs.msg import SmartServoStatus
from inmoov_msgs.msg import MotorCommand
from inmoov_msgs.srv import MotorParameter

from std_msgs.msg import Header

import os
import sys
from os.path import dirname, abspath

#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(abspath(__file__))),'include'))

from constants import PROTOCOL
from servos import Servo
from load_config_from_param import load_config_from_param

servos = {}
lookup = {}

joints = {}
bus = {}

state = {}

sensorpos = 0.0
position = 0.0

smartservostatus = SmartServoStatus()

def init():
    
    rospy.init_node('servo_adc_tester', anonymous=False)
    rate = rospy.Rate(40) # 40hz

    servos = load_config_from_param()

    #now, load lookup name by (bus*255+servo id)
    for n,s in servos.items():
        try:
            key = ((int(s.bus)*255)+int(s.servoPin))
        except:
            rospy.logwarn('motor_status_dispatcher:  unknown servo at bus:'+str(s.bus)+' servo:'+str(s.servoPin))
        
        lookup[key] = n
        print 'key:  ' + str(key)

    servicebus = {}
    commandbus = {}

    # iterate through servo collection
    for j,b in rospy.get_param('/joints').items():
        
        number = rospy.get_param('/joints/' + j + '/bus')
        commandbusname = '/servobus/' + str(number).zfill(2) + '/motorcommand'
        servicebusname = '/servobus/' + str(number).zfill(2) + '/motorparameter'

        # index by busnumber, add to collection if busnumber not found in collection
        if not servicebus.has_key(number):
            commandbus[number] = rospy.Publisher(commandbusname, MotorCommand, queue_size=10)
            servicebus[number] = rospy.ServiceProxy(servicebusname, MotorParameter)

    motorcommand = MotorCommand()

    for n,s in servos.items():
        rospy.loginfo('loading servobrain:  ' + n)
        motorcommand.id = s.servoPin
        motorcommand.parameter = PROTOCOL.MINPULSE
        motorcommand.value = s.minPulse
        commandbus[s.bus].publish(motorcommand)

        motorcommand.parameter = PROTOCOL.MAXPULSE
        motorcommand.value = s.maxPulse
        commandbus[s.bus].publish(motorcommand)

        motorcommand.parameter = PROTOCOL.MINGOAL
        motorcommand.value = s.minGoal
        commandbus[s.bus].publish(motorcommand)

        motorcommand.parameter = PROTOCOL.MAXGOAL
        motorcommand.value = s.maxGoal
        commandbus[s.bus].publish(motorcommand)

        motorcommand.parameter = PROTOCOL.MINSENSOR
        motorcommand.value = s.minSensor
        commandbus[s.bus].publish(motorcommand)

        motorcommand.parameter = PROTOCOL.MAXSENSOR
        motorcommand.value = s.maxSensor
        commandbus[s.bus].publish(motorcommand)



        

def dispatcher(data):

    global sensorpos
    
    sensorpos = float(data.position)
    #print("SENSOR:" + str(sensorpos))

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass