#!/usr/bin/env python

import sys
import os
import rospy
import rospkg

import random

from threading import Thread
import thread
import atexit

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding.QtWidgets import QWidget

from PyQt5 import QtWidgets, QtCore, uic

#from lookgui import Ui_MainWindow

from inmoov_msgs.msg import MotorStatus
from inmoov_msgs.msg import MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import os
import sys
from os.path import dirname, abspath

#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(dirname(abspath(__file__)))),'include'))

from constants import PROTOCOL
from servos import Servo

from time import sleep

# https://github.com/ColinDuquesnoy/QDarkStyleSheet
import qdarkstyle

# https://www.safaribooksonline.com/blog/2014/01/22/create-basic-gui-using-pyqt/
gui = os.path.join(os.path.dirname(__file__), 'enabler.ui')
form_class = uic.loadUiType(gui)[0]

# https://nikolak.com/pyqt-qt-designer-getting-started/
#class ExampleApp(QtGui.QMainWindow, Ui_MainWindow):
class ExampleApp(QtWidgets.QMainWindow, form_class):
    def __init__(self):
        # Explaining super is out of the scope of this article
        # So please google it if you're not familar with it
        # Simple reason why we use it here is that it allows us to
        # access variables, methods etc in the design.py file
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
        # It sets up layout and widgets that are defined

        self.servos = {}

        self.bus = {}

        self.parameterTopic = ["servobus/torso/motorparameter","servobus/leftarm/motorparameter","servobus/rightarm/motorparameter"]

        self.motorcommand = MotorCommand()
        self.jointcommand = JointState()
        
        self.jointNames = []
        
        rospy.init_node('enabler', anonymous=True)

        print("INITIALIZED")

        self.load_config_from_param()

        for j,b in rospy.get_param('/joints').items():
        
            number = rospy.get_param('/joints/' + j + '/bus')
            busname = '/servobus/' + str(number).zfill(2) + '/motorcommand'

            if not self.bus.has_key(number):
                self.bus[number] = rospy.Publisher(busname, MotorCommand, queue_size=10)
                rospy.loginfo('adding:  ' + busname)

        self.commandPublisher = []
        self.commandPublisher.append(rospy.Publisher("servobus/torso/motorcommand", MotorCommand, queue_size=10))
        self.commandPublisher.append(rospy.Publisher("servobus/leftarm/motorcommand", MotorCommand, queue_size=10))
        self.commandPublisher.append(rospy.Publisher("servobus/rightarm/motorcommand", MotorCommand, queue_size=10))

        print("COMMANDS COMPLETE")

        #self.statusSubscriber = []
        #self.statusSubscriber.append(rospy.Subscriber("servobus/torso/motorstatus", MotorStatus, self.callback0))
        #self.statusSubscriber.append(rospy.Subscriber("servobus/leftarm/motorstatus", MotorStatus, self.callback1))
        #self.statusSubscriber.append(rospy.Subscriber("servobus/rightarm/motorstatus", MotorStatus, self.callback2))
        
        print("SUBSCRIBER COMPLETE")

        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)
        
        print("JOINTPUBLISHER COMPLETE")

        self.btnEnableAll.clicked.connect(self.setEnableAll)
        self.btnDisableAll.clicked.connect(self.setDisableAll)

        print("INIT COMPLETE")  

    def setEnableAll(self):
        for j,s in self.servos.items():


            motorcommand = MotorCommand()
            motorcommand.id = int(s.servo)
            motorcommand.parameter = PROTOCOL.ENABLE
            motorcommand.value = 1

            print self.bus[s.bus]

            self.bus[s.bus].publish(motorcommand)



    def setDisableAll(self):
        for j,s in self.servos.items():

            motorcommand = MotorCommand()
            motorcommand.id = int(s.servo)
            motorcommand.parameter = PROTOCOL.ENABLE
            motorcommand.value = 0

            print self.bus[s.bus]

            self.bus[s.bus].publish(motorcommand)


    def closeEvent(self, event):
        self.enabled = False
        self.random = False
        print "GOODBYE!"

    def load_config_from_param(self):

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

            self.servos[name] = s

        print "DONE"

def main():
    app = QtWidgets.QApplication(sys.argv)  # A new instance of QApplication
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    form = ExampleApp()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app
   
if __name__ == '__main__':  # if we're running file directly and not importing it
    main()

