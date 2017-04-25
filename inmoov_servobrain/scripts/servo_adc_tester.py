#!/usr/bin/env python
# licensed under BSD-3


import rospy

from random import randint

from inmoov_msgs.msg import SmartServoStatus
from inmoov_msgs.msg import MotorCommand

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

    commands = rospy.Publisher("/servobus/00/motorcommand", MotorCommand, queue_size=10)

    status = rospy.Subscriber("/servobus/00/smartservostatus", SmartServoStatus, dispatcher)

    x = 0

    while x < (180 * 2 * 10):  # sample size is 3600 samples
        # get random angle between 0 and 180, 0.5 degree increments
        a = float(randint(0,360)) / 2.0

        motorcommand = MotorCommand()
        motorcommand.id = 8
        motorcommand.parameter = PROTOCOL.GOAL
        motorcommand.value = a

        position = float(a)

        commands.publish(motorcommand)
        rospy.sleep(2)
        print(str(a) + "," + str(sensorpos)) 

        x += 1


    #rospy.spin()

def dispatcher(data):

    global sensorpos
    
    sensorpos = float(data.position)
    #print("SENSOR:" + str(sensorpos))

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass