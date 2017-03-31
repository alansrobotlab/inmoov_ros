#!/usr/bin/env python
# licensed as BSD-3

import sys
import rospy
import yaml
import os
from os.path import dirname, abspath

#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(abspath(__file__))),'include'))

from constants import Protocol
from servos import Servo

from inmoov_msgs.srv import MotorParameter

servos = {}

def exporter():

    rospy.init_node("generate_export_yaml", log_level=rospy.INFO)

    load_config_from_param()

    check_config()

    update_config_from_eeprom()

    emit_export_yaml()

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

        s.servopin  = motorparameter(int(s.servo), Protocol.SERVOPIN).data
        s.sensorpin = motorparameter(int(s.servo), Protocol.SENSORPIN).data
        s.minpulse  = motorparameter(int(s.servo), Protocol.MINPULSE).data
        s.maxpulse  = motorparameter(int(s.servo), Protocol.MAXPULSE).data
        s.minangle  = motorparameter(int(s.servo), Protocol.MINANGLE).data
        s.maxangle  = motorparameter(int(s.servo), Protocol.MAXANGLE).data
        s.minsensor = motorparameter(int(s.servo), Protocol.MINSENSOR).data
        s.maxsensor = motorparameter(int(s.servo), Protocol.MAXSENSOR).data
        s.maxspeed  = motorparameter(int(s.servo), Protocol.MAXSPEED).data
        s.smoothing = motorparameter(int(s.servo), Protocol.SMOOTH).data

    print "DONE"

def emit_export_yaml():

    # this is the fastest way to do it, but doesn't preserve the original yaml format
    # wouldn't be hard to rewrite the function to bigbang the yaml file
    # in the exact format of config.yaml

    infile = os.path.join(dirname(dirname(abspath(__file__))),'config','config.yaml')
    outfile = os.path.join(dirname(dirname(abspath(__file__))),'config','export.yaml')
    print infile
    print outfile

    y = yaml.load(open(infile, 'r'))

    for name in servos:
        s = servos[name]
        print "updating yaml for:  " +  name

        y['joints'][name]['bus'        ] = int(s.bus)
        y['joints'][name]['servo'      ] = int(s.servo)
        y['joints'][name]['flip'       ] = bool(s.flip)
        y['joints'][name]['servopin'   ] = int(s.servopin)
        y['joints'][name]['sensorpin'  ] = int(s.sensorpin)
        y['joints'][name]['minpulse'   ] = int(s.minpulse)
        y['joints'][name]['maxpulse'   ] = int(s.maxpulse)
        y['joints'][name]['minangle'   ] = s.minangle
        y['joints'][name]['maxangle'   ] = s.maxangle
        y['joints'][name]['minsensor'  ] = int(s.minsensor)
        y['joints'][name]['maxsensor'  ] = int(s.maxsensor)
        y['joints'][name]['smoothing'  ] = int(s.smoothing)
        y['joints'][name]['maxspeed'   ] = s.maxspeed

    with open(outfile, 'w') as yaml_file:
        yaml_file.write( yaml.dump(y, default_flow_style=False))


    print "DONE"


if __name__ == '__main__':
    try:
        exporter()
    except rospy.ROSInterruptException:
        pass