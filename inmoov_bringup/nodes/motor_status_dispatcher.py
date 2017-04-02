#!/usr/bin/env python
# licensed under BSD-3


import rospy

from inmoov_msgs.msg import MotorStatus

from std_msgs.msg import Header

import os
import sys
from os.path import dirname, abspath

#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(abspath(__file__))),'include'))

from constants import PROTOCOL
from servos import Servo

servos = {}
lookup = {}

joints = {}
bus = {}

state = {}

publisher = rospy.Publisher("motor_status", MotorStatus, queue_size=10)

def init():
    
    rospy.init_node('motor_status_dispatcher', anonymous=False)
    rate = rospy.Rate(40) # 40hz

    load_config_from_param()

    #now, load lookup name by (bus*255+servo id)
    for n,s in servos.items():
        try:
            key = ((int(s.bus)*255)+int(s.servo))
        except:
            rospy.logwarn('motor_status_dispatcher:  unknown servo at bus:'+str(s.bus)+' servo:'+str(s.servo))
        
        lookup[key] = n
        print 'key:  ' + str(key)

    #publisher = rospy.Publisher("motor_status", MotorStatus, queue_size=10)

    for j,b in rospy.get_param('/joints').items():
    
        #create motorstatus bus name
        number = rospy.get_param('/joints/' + j + '/bus')
        busname = '/servobus/' + str(number).zfill(2) + '/motorstatus'

        # and if it's not already in the bus{}, then add it
        # (not sure if the check is required)
        if not bus.has_key(number):
            bus[number] = rospy.Subscriber(busname, MotorStatus, dispatcher, (number))
            rospy.loginfo('adding:  ' + busname)

    rospy.spin()

def dispatcher(data, bus):

    try:
        jointname = lookup[((int(bus)*255)+int(data.id))]
        data.joint = jointname
        data.bus = bus
        publisher.publish(data)
    except:
        rospy.logwarn('motor_status_dispatcher:  unknown servo at bus:'+str(bus)+' servo:'+str(data.id))
    


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