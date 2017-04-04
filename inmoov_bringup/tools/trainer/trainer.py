#!/usr/bin/env python

import sys
import os
import rospy
import rospkg

from PyQt5 import QtWidgets, QtCore, uic

from threading import Thread

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding.QtWidgets import QWidget
#from trainergui import Ui_MainWindow

from inmoov_msgs.msg import MotorStatus
from inmoov_msgs.msg import MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from time import sleep

import os
import sys
from os.path import dirname, abspath

# https://github.com/ColinDuquesnoy/QDarkStyleSheet
import qdarkstyle

#Ui_MainWindow, QtBaseClass = uic.loadUiType("trainer.ui")
# https://www.safaribooksonline.com/blog/2014/01/22/create-basic-gui-using-pyqt/
gui = os.path.join(os.path.dirname(__file__), 'trainer.ui')
Ui_MainWindow, QtBaseClass = uic.loadUiType(gui)

#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(dirname(abspath(__file__)))),'include'))

from constants import PROTOCOL
from servos import Servo

# https://nikolak.com/pyqt-qt-designer-getting-started/
class TrainerApp(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        # Explaining super is out of the scope of this article
        # So please google it if you're not familar with it
        # Simple reason why we use it here is that it allows us to
        # access variables, methods etc in the design.py file
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
        # It sets up layout and widgets that are defined

        self.servos = {}
        self.load_config_from_param()

        self.servicebus = {}
        self.commandbus = {}

        self.joint = Servo()  # which joint is currently selected

        self.values = {}

        # iterate through servo collection
        for j,b in rospy.get_param('/joints').items():
            
            number = rospy.get_param('/joints/' + j + '/bus')
            commandbusname = '/servobus/' + str(number).zfill(2) + '/motorcommand'
            servicebusname = '/servobus/' + str(number).zfill(2) + '/motorparameter'

            # index by busnumber, add to collection if busnumber not found in collection
            if not self.servicebus.has_key(number):
                self.commandbus[number] = rospy.Publisher(commandbusname, MotorCommand, queue_size=10)
                self.servicebus[number] = rospy.ServiceProxy(servicebusname, MotorParameter)

        self.setupDropDowns()

        self.motorcommand = MotorCommand()
        self.motorparameter = MotorParameter()
        self.jointcommand = JointState()
        
        self.jointNames = []

        rospy.init_node('trainer', anonymous=True)

        print("INITIALIZED")

        self.statusSubscriber = rospy.Subscriber("motor_status", MotorStatus, self.motorstatus)
        
        print("SUBSCRIBER COMPLETE")

        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)
        
        print("JOINTPUBLISHER COMPLETE")

        self.servo = 0

        self.connectUI()
        self.servoChanged()
        
        print("INIT COMPLETE")

    def connectUI(self):
        self.cmbServo.currentIndexChanged.connect(self.servoChanged)

        self.txtGoal.editingFinished.connect(lambda: self.valueChanged(self.txtGoal, PROTOCOL.GOALPOSITION))
        self.txtMinPulse.editingFinished.connect(lambda: self.valueChanged(self.txtMinPulse, PROTOCOL.MINPULSE))
        self.txtMaxPulse.editingFinished.connect(lambda: self.valueChanged(self.txtMaxPulse, PROTOCOL.MAXPULSE))
        self.txtMinGoal.editingFinished.connect(lambda: self.valueChanged(self.txtMinGoal, PROTOCOL.MINANGLE))
        self.txtMaxGoal.editingFinished.connect(lambda: self.valueChanged(self.txtMaxGoal, PROTOCOL.MAXANGLE))
        self.txtMaxSpeed.editingFinished.connect(lambda: self.valueChanged(self.txtMaxSpeed, PROTOCOL.MAXSPEED))
        self.txtMinSensor.editingFinished.connect(lambda: self.valueChanged(self.txtMinSensor, PROTOCOL.MINSENSOR))
        self.txtMaxSensor.editingFinished.connect(lambda: self.valueChanged(self.txtMaxSensor, PROTOCOL.MAXSENSOR))
        self.chkEnabled.stateChanged.connect(lambda: self.valueChanged(self.chkEnabled, PROTOCOL.ENABLE))
        self.chkCalibrated.stateChanged.connect(lambda: self.valueChanged(self.chkCalibrated, PROTOCOL.CALIBRATED))
        
        self.sliderGoal.valueChanged.connect(lambda: self.valueChanged(self.sliderGoal, PROTOCOL.GOALPOSITION))

    def disconnectUI(self):
        self.cmbServo.currentIndexChanged.disconnect()

        self.txtGoal.editingFinished.disconnect()
        self.txtMinPulse.editingFinished.disconnect()
        self.txtMaxPulse.editingFinished.disconnect()
        self.txtMinGoal.editingFinished.disconnect()
        self.txtMaxGoal.editingFinished.disconnect()
        self.txtMaxSpeed.editingFinished.disconnect()
        self.txtMinSensor.editingFinished.disconnect()
        self.txtMaxSensor.editingFinished.disconnect()
        self.chkEnabled.stateChanged.disconnect()
        self.chkCalibrated.stateChanged.disconnect()
        
        self.sliderGoal.valueChanged.disconnect()


    def servoChanged(self):
    
        print("SERVOCHANGED")

        self.disconnectUI()

        self.joint = self.servos[self.cmbServo.currentText()]

        val = str(round(self.getParameter(PROTOCOL.GOALPOSITION ),3))
        self.txtGoal.setText(val)

        val = str(int(self.getParameter(PROTOCOL.SERVOPIN )))
        self.txtServoPin.setText(val)

        val = str(int(self.getParameter(PROTOCOL.SENSORPIN )))
        self.txtSensorPin.setText(val)

        val = str(int(self.getParameter(PROTOCOL.MINPULSE )))
        self.txtMinPulse.setText(val)

        val = str(int(self.getParameter(PROTOCOL.MAXPULSE )))
        self.txtMaxPulse.setText(val)

        val = str(round(self.getParameter(PROTOCOL.MINANGLE ),3))
        self.txtMinGoal.setText(val)

        val = str(round(self.getParameter(PROTOCOL.MAXANGLE ),3))
        self.txtMaxGoal.setText(val)

        val = str(round(self.getParameter(PROTOCOL.MAXSPEED ),3))
        self.txtMaxSpeed.setText(val)

        val = str(int(self.getParameter(PROTOCOL.MINSENSOR )))
        self.txtMinSensor.setText(val)

        val = str(int(self.getParameter(PROTOCOL.MAXSENSOR )))
        self.txtMaxSensor.setText(val)

        # now set up the slider
        minval = self.values[PROTOCOL.MINANGLE]
        maxval = self.values[PROTOCOL.MAXANGLE]
        goal   = self.values[PROTOCOL.GOALPOSITION]
        
        if minval < maxval :
            self.sliderGoal.setMinimum(int(minval * 1000.0))
            self.sliderGoal.setMaximum(int(maxval * 1000.0))
        else:
            self.sliderGoal.setMinimum(int(maxval * 1000.0))
            self.sliderGoal.setMaximum(int(minval * 1000.0))

        self.sliderGoal.setValue(int(goal)*1000.0)


        self.connectUI()

    
    def getParameter(self, parameter):

        j = self.joint

        try:
            val = self.servicebus[j.bus](j.servo, parameter).data
            self.values[parameter] = val
            print val
            return val
        except:
            return 0.0

    def setParameter(self, parameter, value):

        j = self.joint

        # send to arduino directly
        try:
            motorcommand = MotorCommand()
            motorcommand.id = int(j.servo)
            motorcommand.parameter = parameter
            motorcommand.value = value

            self.commandbus[j.bus].publish(motorcommand)
        except:
            rospy.logwarn('trainer:  bad parameter or something.')

        # send to parameter server

        # record to config.yaml?

        # update config.xacro?

    def valueChanged(self, field, parameter):
        if field.metaObject().className() == 'QLineEdit':
            if field.text() != self.values[parameter]:
                print field.text()
                self.setParameter(parameter, float(field.text()))
                self.values[parameter] = field.text()
            if field.objectName() == 'txtGoal':
                self.sliderGoal.setValue(float(self.values[PROTOCOL.GOALPOSITION])*1000.0)
            return

        if field.metaObject().className() == 'QCheckBox':
            field.blockSignals(True)
            self.setParameter(parameter, float(field.isChecked()))
            field.blockSignals(False)
            return

        if field.metaObject().className() == 'QSlider':
            val = float(field.value())/1000.0
            self.setParameter(parameter, val)
            self.values[parameter] = str(round(val,3))
            self.txtGoal.setText(str(round(val,3)))
            return

    def motorstatus(self, data):
        if data.joint == self.cmbServo.currentText():
            self.txtSensorRaw.setText(str(int(data.posraw)))
            self.txtPosition.setText(str(round(data.position,3)))
            self.txtSpeed.setText(str(data.presentspeed))

            # these don't work yet
            self.chkEnabled.setChecked(data.enabled)
            self.chkMoving.setChecked(data.moving)
            self.chkPower.setChecked(data.power)

    
    def degreestoradians(self, d):
        return d*(3.1415926/180.0)
    
    def setupDropDowns(self):

        for name in self.servos:
            self.cmbServo.addItem(name)

        self.cmbServo.model().sort(0)
        self.cmbServo.setCurrentIndex(0)

        self.cmbSmoothing.addItem('0 - Instant')
        self.cmbSmoothing.addItem('1 - Max Speed')
        self.cmbSmoothing.addItem('2 - Linear Ramp')
        self.cmbSmoothing.addItem('3 - COS Ramp')
        self.cmbSmoothing.addItem('4 - COS^2 Ramp')


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

            self.servos[name] = s

def clamp(n,minn,maxn):
    return max(min(maxn, n), minn)



def main():
    app = QtWidgets.QApplication(sys.argv)  # A new instance of QApplication
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    form = TrainerApp()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app
   
if __name__ == '__main__':  # if we're running file directly and not importing it
    main()
