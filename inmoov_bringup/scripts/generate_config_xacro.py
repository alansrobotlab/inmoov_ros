#!/usr/bin/env python
# licensed as BSD-3

import sys
import rospy
import yaml
import os
from os.path import dirname, abspath

#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(abspath(__file__))),'include'))

from constants import PROTOCOL
from servos import Servo

from inmoov_msgs.srv import MotorParameter

servos = {}

def exporter():

    rospy.init_node("generate_config_xacro", log_level=rospy.INFO)

    load_config_from_param()

    check_config()

    update_config_from_eeprom()

    emit_config_xacro()

    print "DONE!"



def load_config_from_param():

    # first, make sure parameter server is even loaded
    while not rospy.search_param("/joints"):
        rospy.loginfo("waiting for parameter server to load with joint definitions")
        rospy.sleep(1)

    rospy.sleep(1)

    joints = rospy.get_param('/joints')
    for name in joints:
        print "found:  " + name

        s = Servo()

        key = '/joints/' + name + '/'

        s.bus       =  rospy.get_param(key + 'bus')
        s.servo     =  rospy.get_param(key + 'servo')
        s.flip      =  rospy.get_param(key + 'flip')

        s.servopin  =  rospy.get_param(key + 'servopin')
        s.sensorpin =  rospy.get_param(key + 'sensorpin')
        s.minpulse  =  rospy.get_param(key + 'minpulse')
        s.maxpulse  =  rospy.get_param(key + 'maxpulse')
        s.minangle  =  rospy.get_param(key + 'minangle')
        s.maxangle  =  rospy.get_param(key + 'maxangle')
        s.minsensor =  rospy.get_param(key + 'minsensor')
        s.maxsensor =  rospy.get_param(key + 'maxsensor')
        s.maxspeed  =  rospy.get_param(key + 'maxspeed')
        s.smoothing =  rospy.get_param(key + 'smoothing')

        servos[name] = s


    print "DONE"

def check_config():
    print "DONE"

def update_config_from_eeprom():

    for name in servos:
        s = servos[name]
        print "interrogating eeprom for:  " + name

        rospy.wait_for_service("servobus/" + str(s.bus).zfill(2) + "/motorparameter")

        motorparameter = rospy.ServiceProxy("servobus/" + str(s.bus).zfill(2) + "/motorparameter", MotorParameter)

        s.servopin  = motorparameter(int(s.servo), PROTOCOL.SERVOPIN).data
        s.sensorpin = motorparameter(int(s.servo), PROTOCOL.SENSORPIN).data
        s.minpulse  = motorparameter(int(s.servo), PROTOCOL.MINPULSE).data
        s.maxpulse  = motorparameter(int(s.servo), PROTOCOL.MAXPULSE).data
        s.minangle  = motorparameter(int(s.servo), PROTOCOL.MINANGLE).data
        s.maxangle  = motorparameter(int(s.servo), PROTOCOL.MAXANGLE).data
        s.minsensor = motorparameter(int(s.servo), PROTOCOL.MINSENSOR).data
        s.maxsensor = motorparameter(int(s.servo), PROTOCOL.MAXSENSOR).data
        s.maxspeed  = motorparameter(int(s.servo), PROTOCOL.MAXSPEED).data
        s.smoothing = motorparameter(int(s.servo), PROTOCOL.SMOOTH).data

    print "DONE"

def emit_config_xacro():

    # this is the fastest way to do it, but doesn't preserve the original yaml format
    # wouldn't be hard to rewrite the function to bigbang the yaml file
    # in the exact format of config.yaml

    #infile = os.path.join(dirname(dirname(abspath(__file__))),'config','config.yaml')
    outfile = os.path.join(dirname(dirname(abspath(__file__))),'config','config.xacro')
    #print infile
    print outfile

    #y = yaml.load(open(infile, 'r'))

    with open(outfile, 'w') as xacro:
        xacro.write('<?xml version="1.0"?>' + '\n')
        xacro.write('\n')
        xacro.write('<robot xmlns:xacro="http://ros.org/wiki/xacro">' + '\n')
        xacro.write('\n')

        for name in servos:
            s = servos[name]
            print "updating yaml for:  " +  name
            
            xacro.write('<xacro:property name="' + name + '_lower"    value="${pi*' + str(round(s.minangle,3)) +'/180.0}" /> \n')
            xacro.write('<xacro:property name="' + name + '_upper"    value="${pi*' + str(round(s.maxangle,3)) +'/180.0}" /> \n')
            xacro.write('<xacro:property name="' + name + '_velocity" value="${pi*' + str(round(s.maxspeed,3)) +'/180.0}" /> \n')

            xacro.write('\n')

        xacro.write('</robot>')

        xacro.close()

    print "DONE"


if __name__ == '__main__':
    try:
        exporter()
    except rospy.ROSInterruptException:
        pass