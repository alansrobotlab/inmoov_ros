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

from export_yaml import export_yaml
from load_config_from_param import load_config_from_param

from inmoov_msgs.srv import MotorParameter

servos = {}

def exporter():

    rospy.init_node("generate_export_yaml", log_level=rospy.INFO)

    servos = load_config_from_param()

    check_config()

    update_config_from_eeprom(servos)

    export_yaml('export.yaml')

    print "DONE!"

def check_config():
    print "DONE"

def update_config_from_eeprom(servos):

    for name in servos:
        s = servos[name]
        print "interrogating eeprom for:  " + name

        rospy.wait_for_service("servobus/" + str(s.bus).zfill(2) + "/motorparameter")

        motorparameter = rospy.ServiceProxy("servobus/" + str(s.bus).zfill(2) + "/motorparameter", MotorParameter)

        key = '/joints/' + name + '/'

        val  = motorparameter(int(s.servo), PROTOCOL.SERVOPIN).data
        rospy.set_param(key + 'servoPin', val)
        val = motorparameter(int(s.servo), PROTOCOL.SENSORPIN).data
        rospy.set_param(key + 'sensorPin', val)
        val  = motorparameter(int(s.servo), PROTOCOL.MINPULSE).data
        rospy.set_param(key + 'minPulse', val)
        val  = motorparameter(int(s.servo), PROTOCOL.MAXPULSE).data
        rospy.set_param(key + 'maxPulse', val)
        val   = motorparameter(int(s.servo), PROTOCOL.MINGOAL).data
        rospy.set_param(key + 'minGoal', val)
        val   = motorparameter(int(s.servo), PROTOCOL.MAXGOAL).data
        rospy.set_param(key + 'maxGoal', val)
        val      = motorparameter(int(s.servo), PROTOCOL.REST).data
        rospy.set_param(key + 'rest', val)
        val = motorparameter(int(s.servo), PROTOCOL.MINSENSOR).data
        rospy.set_param(key + 'minSensor', val)
        val = motorparameter(int(s.servo), PROTOCOL.MAXSENSOR).data
        rospy.set_param(key + 'maxSensor', val)
        val  = motorparameter(int(s.servo), PROTOCOL.MAXSPEED).data
        rospy.set_param(key + 'maxSpeed', val)


    print "DONE"


if __name__ == '__main__':
    try:
        exporter()
    except rospy.ROSInterruptException:
        pass