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

samples = [0.0, 0.0, 0.0, 0.0]
sampleposition = 0
moving = True

feedback = -1.0
temperature = 0
position = 0.0

MINGOAL = 10
MAXGOAL = 170

# this is a hand tuned value based on the test rig
# 170-10 = 160 angle range, 1686-409 = 1277 feedback range
# 160/1277 = ~0.125 degrees per gradation
# so, with samples[4], if one sample is off by one gradation
# that is ~ 0.031, so we'll round up to 0.035
# as long as 3 of 4 match, and 1 is off by one, we'll accept
SETTLE_ERROR = 0.035

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
    global temperature

    with open('output.csv', 'w') as export:

        export.write("count,temperature,command,error,feedback")

        while x < ((MAXGOAL-MINGOAL) * 2 * SAMPLES):  # sample size is 3600 samples
            # get random angle between 0 and 180, 0.5 degree increments
            a = float(randint(MINGOAL * 2,MAXGOAL * 2)) / 2.0

            motorcommand = MotorCommand()
            motorcommand.id = 8
            motorcommand.parameter = PROTOCOL.GOAL
            motorcommand.value = a

            position = float(a)

            commands.publish(motorcommand)

            while moving == True:
                rospy.sleep(0.1)

            output = "{:6.0f}".format(x) + "," + "{:2.0f}".format(temperature) + ","  + "{:7.2f}".format(a) + "," + "{:7.2f}".format(feedback -a) + "," + "{:7.2f}".format(feedback)
            export.write(output + '\n')
            print(output) 

            x += 1
            moving = True

    export.close()

    #rospy.spin()

def dispatcher(data):

    global feedback
    global temperature
    global samples
    global sampleposition
    global moving
    
    feedback = float(data.position)
    temperature = data.temp
    samples[sampleposition] = feedback
    sampleposition = (sampleposition + 1) & 3

    avg = (samples[0] + samples[1] + samples[2] + samples[3])/4

    #print(abs(feedback - avg))

    if abs(feedback - avg) > SETTLE_ERROR:  # hand tuned value
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