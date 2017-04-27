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

samples = [0.0,0.0,0.0,0.0]
sampleposition = 0
moving = True

feedback = 0.0
position = 0.0

MINGOAL = 10
MAXGOAL = 170

SETTLE = 2.0

SAMPLES = 30

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

    global moving

    with open('output.csv', 'w') as export:

        while x < ((MAXGOAL-MINGOAL) * 2 * SAMPLES):  # sample size is 3600 samples
            # get random angle between 0 and 180, 0.5 degree increments
            a = float(randint(MINGOAL,MAXGOAL * 2)) / 2.0

            motorcommand = MotorCommand()
            motorcommand.id = 8
            motorcommand.parameter = PROTOCOL.GOAL
            motorcommand.value = a

            position = float(a)

            commands.publish(motorcommand)

            while moving == True:
                rospy.sleep(0.1)

            output = str(a) + "," + str(feedback)
            export.write(output + '\n')
            print(output) 

            x += 1
            moving = True

    export.close()

    #rospy.spin()

def dispatcher(data):

    global feedback
    global samples
    global sampleposition
    global moving
    
    feedback = float(data.position)
    samples[sampleposition] = feedback
    sampleposition = (sampleposition + 1) & 3

    avg = (samples[0] + samples[1] + samples[2] + samples[3])/4

    #print(abs(feedback - avg))

    if abs(feedback - avg) > 0.1:
        moving = True
        #print(abs(feedback - avg))
    else:
        moving = False


    #print("SENSOR:" + str(sensorpos))

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass