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
    while not rospy.search_param("/joints/angles"):
        rospy.loginfo("waiting for parameter server to load with joint definitions")
        rospy.sleep(1)

    rospy.sleep(1)

    i = 0

    while rospy.search_param('/joints/joint' + str(i).zfill(2)):

        param = '/joints/joint' + str(i).zfill(2)

        print "found " + str(i).zfill(2)

        servo = Servo()
        servo.key       =  i
        servo.name      =  rospy.get_param(param + '/name')
        servo.bus       =  rospy.get_param(param + '/bus')
        servo.servo     =  rospy.get_param(param + '/servo')
        servo.flip      =  rospy.get_param(param + '/flip')

        servo.servopin  =  rospy.get_param(param + '/servopin')
        servo.sensorpin =  rospy.get_param(param + '/sensorpin')
        servo.minpulse  =  rospy.get_param(param + '/minpulse')
        servo.maxpulse  =  rospy.get_param(param + '/maxpulse')
        servo.minangle  =  rospy.get_param(param + '/minangle')
        servo.maxangle  =  rospy.get_param(param + '/maxangle')
        servo.minsensor =  rospy.get_param(param + '/minsensor')
        servo.maxsensor =  rospy.get_param(param + '/maxsensor')
        servo.maxspeed  =  rospy.get_param(param + '/maxspeed')
        servo.smoothing =  rospy.get_param(param + '/smoothing')

        servos[servo.name] = servo

        i+=1

    #print servos

    print "DONE"

def check_config():
    print "DONE"

def update_config_from_eeprom():

    for name in servos:
        s = servos[name]
        print s.name

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
        print s.name

        joint = 'joint' + str(s.key).zfill(2)

        y['joints'][joint]['bus'        ] = int(s.bus)
        y['joints'][joint]['servo'      ] = int(s.servo)
        y['joints'][joint]['flip'       ] = bool(s.flip)
        y['joints'][joint]['servopin'   ] = int(s.servopin)
        y['joints'][joint]['sensorpin'  ] = int(s.sensorpin)
        y['joints'][joint]['minpulse'   ] = int(s.minpulse)
        y['joints'][joint]['maxpulse'   ] = int(s.maxpulse)
        y['joints'][joint]['minangle'   ] = s.minangle
        y['joints'][joint]['maxangle'   ] = s.maxangle
        y['joints'][joint]['minsensor'  ] = int(s.minsensor)
        y['joints'][joint]['maxsensor'  ] = int(s.maxsensor)
        y['joints'][joint]['smoothing'  ] = int(s.smoothing)
        y['joints'][joint]['maxspeed'   ] = s.maxspeed

    with open(outfile, 'w') as yaml_file:
        yaml_file.write( yaml.dump(y, default_flow_style=False))


    print "DONE"


if __name__ == '__main__':
    try:
        exporter()
    except rospy.ROSInterruptException:
        pass