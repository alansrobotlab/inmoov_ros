#!/usr/bin/env python
# licensed under BSD-3

servos = {}     # servo configuration data for robot

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
