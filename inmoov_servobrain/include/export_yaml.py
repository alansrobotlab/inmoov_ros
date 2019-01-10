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

def export_yaml(filename):

    outfile = os.path.join(dirname(dirname(abspath(__file__))),'config',filename)

    print outfile

    with open(outfile, 'w') as export:
        export.write('bringup:\n')
        export.write(str('  angles:').ljust(20) + rospy.get_param('/bringup/angles')+ '\n')
        export.write(str('  hz:').ljust(20) + str(rospy.get_param('/bringup/hz')) +'\n')
        export.write('\n')
        export.write('joints:\n')
        export.write('\n')

        for name in rospy.get_param('/joints'):
            print "updating yaml for:  " +  name
            key = '/joints/' + name + '/'

            export.write(str('  ' + name + ':').ljust(20) + '\n')
            val = rospy.get_param(key + 'bus')
            export.write(str('    bus:').ljust(20) + str(int(val)) + '\n')
            val = rospy.get_param(key + 'servoPin')
            export.write(str('    servoPin:').ljust(20) + str(int(val)) + '\n')
            val = rospy.get_param(key + 'minPulse')
            export.write(str('    minPulse:').ljust(20) + str(int(val)) + '\n')
            val = rospy.get_param(key + 'maxPulse')
            export.write(str('    maxPulse:').ljust(20) + str(int(val)) + '\n')
            val = rospy.get_param(key + 'minGoal')
            export.write(str('    minGoal:').ljust(20) + str(val) + '\n')
            val = rospy.get_param(key + 'maxGoal')
            export.write(str('    maxGoal:').ljust(20) + str(val) + '\n')
            val = rospy.get_param(key + 'rest')
            export.write(str('    rest:').ljust(20) + str(val) + '\n')
            val = rospy.get_param(key + 'maxSpeed')
            export.write(str('    maxSpeed:').ljust(20) + str(val) + '\n')
            val = rospy.get_param(key + 'smoothing')
            export.write(str('    smoothing:').ljust(20) + str(int(val)) + '\n')
            val = rospy.get_param(key + 'sensorPin')
            export.write(str('    sensorPin:').ljust(20) + str(int(val)) + '\n')
            val = rospy.get_param(key + 'minSensor')
            export.write(str('    minSensor:').ljust(20) + str(int(val)) + '\n')
            val = rospy.get_param(key + 'maxSensor')
            export.write(str('    maxSensor:').ljust(20) + str(int(val)) + '\n')

            export.write('\n')

        export.close()

    print "DONE"