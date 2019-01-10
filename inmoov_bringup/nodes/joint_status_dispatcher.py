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

servos = {}
lookup = {}

joints = {}
bus = {}

state = {}

jointstatus = JointState()

def init():
    
    rospy.init_node('joint_status_dispatcher', anonymous=False)
    rate = rospy.Rate(40) # 40hz

    servos = load_config_from_param()

    #now, load lookup name by (bus*255+servo id)
    for n,s in servos.items():
        key = ((int(s.bus)*255)+int(s.servoPin))
        lookup[key] = n
        print 'key:  ' + str(key)

    publisher = rospy.Publisher("joint_status", JointState, queue_size=10)

    for j,b in rospy.get_param('/joints').items():
    
        #create motorstatus bus name
        number = rospy.get_param('/joints/' + j + '/bus')
        busname = '/servobus/' + str(number).zfill(2) + '/motorstatus'

        # and if it's not already in the bus{}, then add it
        # (not sure if the check is required)
        if not bus.has_key(number):
            bus[number] = rospy.Subscriber(busname, MotorStatus, dispatcher, (number))
            rospy.loginfo('adding:  ' + busname)

    while not rospy.is_shutdown():

        #jointstatus = JointState()
        jointstatus.header = Header()
        jointstatus.header.stamp = rospy.Time.now()
        jointstatus.name = []
        jointstatus.position = []
        jointstatus.velocity = []
        jointstatus.effort = []


        for j,p in joints.items():
            jointstatus.name.append(j)
            jointstatus.position.append(p)

        if jointstatus.name.count > 0:
            publisher.publish(jointstatus)

        joints.clear()
        rate.sleep()

    rospy.spin()

def dispatcher(data, bus):
    try:
        #print "OHAI! bus:" + str(bus) + " servo:" + str(data.id)
        key = lookup[((int(bus)*255)+int(data.id))]
        joints[key] = data.position
    except:
        rospy.logwarn('joint_status_dispatcher:  unknown servo at bus:'+str(bus)+' servo:'+str(data.id))
    



if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass