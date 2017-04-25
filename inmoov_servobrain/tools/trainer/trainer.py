#!/usr/bin/env python

import sys
import os
import rospy
import rospkg

import yaml

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtWidgets import QPushButton, QMessageBox

from threading import Thread

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding.QtWidgets import QWidget
#from trainergui import Ui_MainWindow

from inmoov_msgs.msg import SmartServoStatus
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

from export_yaml import export_yaml
from load_config_from_param import load_config_from_param

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

        self.servos = load_config_from_param()
        #self.load_config_from_param()

        self.servicebus = {}
        self.commandbus = {}

        self.joint = Servo()  # which joint is currently selected
        self.jointName = 'none'       # name of the joint currently selected

        self.positions = {}  # we're going to collect positions from motorstatus
                             # so we can initialize the goal and goal slider
                             # to the last command value

        self.values = {}

        self.saved = True

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

        self.statusSubscriber = rospy.Subscriber("motor_status", SmartServoStatus, self.motorstatus)
        
        print("SUBSCRIBER COMPLETE")

        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)
        
        print("JOINTPUBLISHER COMPLETE")

        self.servo = 0

        self.connectUI()
        self.servoChanged()

        self.btnWriteConfiguration.clicked.connect(self.writeConfiguration)
        
        print("INIT COMPLETE")

    def connectUI(self):
        self.cmbServo.currentIndexChanged.connect(self.servoChanged)

        self.txtGoal.editingFinished.connect(lambda: self.valueChanged(self.txtGoal, PROTOCOL.GOAL))
        self.txtServoPin.editingFinished.connect(lambda: self.valueChanged(self.txtServoPin, PROTOCOL.SERVOPIN))
        self.txtSensorPin.editingFinished.connect(lambda: self.valueChanged(self.txtSensorPin, PROTOCOL.SENSORPIN))
        self.txtMinPulse.editingFinished.connect(lambda: self.valueChanged(self.txtMinPulse, PROTOCOL.MINPULSE))
        self.txtMaxPulse.editingFinished.connect(lambda: self.valueChanged(self.txtMaxPulse, PROTOCOL.MAXPULSE))
        self.txtMinGoal.editingFinished.connect(lambda: self.valueChanged(self.txtMinGoal, PROTOCOL.MINGOAL))
        self.txtMaxGoal.editingFinished.connect(lambda: self.valueChanged(self.txtMaxGoal, PROTOCOL.MAXGOAL))
        self.txtMaxSpeed.editingFinished.connect(lambda: self.valueChanged(self.txtMaxSpeed, PROTOCOL.MAXSPEED))
        self.txtMinSensor.editingFinished.connect(lambda: self.valueChanged(self.txtMinSensor, PROTOCOL.MINSENSOR))
        self.txtMaxSensor.editingFinished.connect(lambda: self.valueChanged(self.txtMaxSensor, PROTOCOL.MAXSENSOR))
        self.chkEnabled.stateChanged.connect(lambda: self.valueChanged(self.chkEnabled, PROTOCOL.ENABLE))
        self.chkCalibrated.stateChanged.connect(lambda: self.valueChanged(self.chkCalibrated, PROTOCOL.CALIBRATED))
        
        self.sliderGoal.valueChanged.connect(lambda: self.valueChanged(self.sliderGoal, PROTOCOL.GOAL))

    def disconnectUI(self):
        self.cmbServo.currentIndexChanged.disconnect()

        self.txtGoal.editingFinished.disconnect()
        self.txtServoPin.editingFinished.disconnect()
        self.txtSensorPin.editingFinished.disconnect()
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

    def writeConfiguration(self):
        self.saved = True
        export_yaml('config.yaml')

    def servoChanged(self):
    
        print("SERVOCHANGED")

        self.disconnectUI()

        self.joint = self.servos[self.cmbServo.currentText()]
        self.jointName = self.cmbServo.currentText()

        val = str(round(self.getParameter(PROTOCOL.GOAL ),3))
        self.txtGoal.setText(val)
        self.values[PROTOCOL.GOAL] = val

        val = str(int(self.getParameter(PROTOCOL.SERVOPIN )))
        self.txtServoPin.setText(val)
        self.values[PROTOCOL.SERVOPIN] = val

        val = str(int(self.getParameter(PROTOCOL.SENSORPIN )))
        self.txtSensorPin.setText(val)
        self.values[PROTOCOL.SENSORPIN] = val

        val = str(int(self.getParameter(PROTOCOL.MINPULSE )))
        self.txtMinPulse.setText(val)
        self.values[PROTOCOL.MINPULSE] = val

        val = str(int(self.getParameter(PROTOCOL.MAXPULSE )))
        self.txtMaxPulse.setText(val)
        self.values[PROTOCOL.MAXPULSE] = val

        val = str(round(self.getParameter(PROTOCOL.MINGOAL ),3))
        self.txtMinGoal.setText(val)
        self.values[PROTOCOL.MINGOAL] = val

        val = str(round(self.getParameter(PROTOCOL.MAXGOAL ),3))
        self.txtMaxGoal.setText(val)
        self.values[PROTOCOL.MAXGOAL] = val

        val = str(round(self.getParameter(PROTOCOL.MAXSPEED ),3))
        self.txtMaxSpeed.setText(val)
        self.values[PROTOCOL.MAXSPEED] = val

        val = str(int(self.getParameter(PROTOCOL.MINSENSOR )))
        self.txtMinSensor.setText(val)
        self.values[PROTOCOL.MINSENSOR] = val

        val = str(int(self.getParameter(PROTOCOL.MAXSENSOR )))
        self.txtMaxSensor.setText(val)
        self.values[PROTOCOL.MAXSENSOR] = val

        # now set up the slider
        minval = float(self.values[PROTOCOL.MINGOAL])
        maxval = float(self.values[PROTOCOL.MAXGOAL])
        goal   = float(self.values[PROTOCOL.GOAL])
        
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
            key = '/joints/' + self.jointName + '/'

            if   parameter == PROTOCOL.SERVOPIN:
                val = rospy.get_param(key + 'servoPin')
            elif parameter == PROTOCOL.SENSORPIN:
                val = rospy.get_param(key + 'sensorPin')
            elif parameter == PROTOCOL.MINGOAL:
                val = rospy.get_param(key + 'minGoal')
            elif parameter == PROTOCOL.MAXGOAL:
                val = rospy.get_param(key + 'maxGoal')
            elif parameter == PROTOCOL.REST:
                val = rospy.get_param(key + 'rest')
            elif parameter == PROTOCOL.MINPULSE:
                val = rospy.get_param(key + 'minPulse')
            elif parameter == PROTOCOL.MAXPULSE:
                val = rospy.get_param(key + 'maxPulse')
            elif parameter == PROTOCOL.MINSENSOR:
                val = rospy.get_param(key + 'minSensor')
            elif parameter == PROTOCOL.MAXSENSOR:
                val = rospy.get_param(key + 'maxSensor')
            elif parameter == PROTOCOL.MAXSPEED:
                val = rospy.get_param(key + 'maxSpeed')
            elif parameter == PROTOCOL.SMOOTHING:
                val = rospy.get_param(key + 'smooth')
            return val
        except:
            return -1.0

    def setParameter(self, parameter, value):

        #try:

            s = self.joint

            # send to arduino directly
            motorcommand = MotorCommand()
            motorcommand.id = int(s.servoPin)
            motorcommand.parameter = parameter
            motorcommand.value = value

            self.commandbus[s.bus].publish(motorcommand)

            # send to joint_command
            self.jointcommand.name = []
            self.jointcommand.position = []

            self.jointcommand.header = Header()
            self.jointcommand.header.stamp = rospy.Time.now()
            self.jointcommand.name.append(self.jointName)
            self.jointcommand.position.append(value)
            self.jointcommand.velocity = []
            self.jointcommand.effort= []
            self.jointPublisher.publish(self.jointcommand)

            # update param server and servo{}
            partial = '/joints/' + self.jointName + '/'

            if   parameter == PROTOCOL.SERVOPIN:
                s.servoPin = value
                rospy.set_param(partial + 'servoPin', value)
                print str(value)
                print str(s.servoPin)
                print rospy.get_param(partial + 'servoPin')
            elif parameter == PROTOCOL.SENSORPIN:
                s.sensorPin = value
                rospy.set_param(partial + 'sensorPin', value)
            elif parameter == PROTOCOL.MINGOAL:
                rospy.set_param(partial + 'minGoal', value)
            elif parameter == PROTOCOL.MAXGOAL:
                rospy.set_param(partial + 'maxGoal', value)
            elif parameter == PROTOCOL.REST:
                rospy.set_param(partial + 'rest', s.rest)
            elif parameter == PROTOCOL.MINPULSE:
                rospy.set_param(partial + 'minPulse', value)
            elif parameter == PROTOCOL.MAXPULSE:
                rospy.set_param(partial + 'maxPulse', value)
            elif parameter == PROTOCOL.MINSENSOR:
                rospy.set_param(partial + 'minSensor', value)
            elif parameter == PROTOCOL.MAXSENSOR:
                rospy.set_param(partial + 'maxSensor', value)
            elif parameter == PROTOCOL.MAXSPEED:
                rospy.set_param(partial + 'maxSpeed', value)
            elif parameter == PROTOCOL.SMOOTHING:
                rospy.set_param(partial + 'smooth', value)

            self.saved = False
        #except:
            #rospy.logwarn('trainer:  bad parameter or something.')

    def valueChanged(self, field, parameter):
        if field.metaObject().className() == 'QLineEdit':
            if field.text() != self.values[parameter]:
                print field.text()
                self.setParameter(parameter, float(field.text()))
                self.values[parameter] = field.text()
            if field.objectName() == 'txtGoal':
                self.sliderGoal.blockSignals(True)
                self.sliderGoal.setValue(float(self.values[PROTOCOL.GOAL])*1000.0)
                self.sliderGoal.blockSignals(False)

            # now set up the slider
            minval = float(self.values[PROTOCOL.MINGOAL])
            maxval = float(self.values[PROTOCOL.MAXGOAL])
            goal   = float(self.values[PROTOCOL.GOAL])
        
            if minval < maxval :
                self.sliderGoal.setMinimum(int(minval * 1000.0))
                self.sliderGoal.setMaximum(int(maxval * 1000.0))
            else:
                self.sliderGoal.setMinimum(int(maxval * 1000.0))
                self.sliderGoal.setMaximum(int(minval * 1000.0))

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
            self.txtGoal.blockSignals(True)
            self.txtGoal.setText(str(round(val,3)))
            self.txtGoal.blockSignals(False)
            return

    def motorstatus(self, data):

        self.positions[data.joint] = float(data.goal)

        if data.joint == self.cmbServo.currentText():
            self.txtSensorRaw.setText(str(int(data.posraw)))
            self.txtPosition.setText(str(round(data.position,3)))
            self.txtSpeed.setText(str(data.presentspeed))
            self.txtTemp.setText(str(data.temp))
            self.txtError.setText(str(data.error))
            self.txtValue1.setText(str(data.value1))
            self.txtValue2.setText(str(data.value2))
            self.txtValue3.setText(str(data.value3))
            self.txtValue4.setText(str(data.value4))
            self.txtValue5.setText(str(data.value5))

            # these don't work yet
            self.chkEnabled.blockSignals(True)
            self.chkEnabled.setChecked(data.enabled)
            self.chkEnabled.blockSignals(False)
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

    def closeEvent(self, event):
        if not self.saved:
            event.ignore()
            buttonReply = QMessageBox.question(self, 'Warning', "You have unsaved parameter Changes!", QMessageBox.Ok, QMessageBox.Ok)
            return
        else:
            self.running = False
            self.enabled = False
            self.random = False
            #self.emit_export_yaml()
            print "GOODBYE!"

def clamp(n,minn,maxn):
    return max(min(maxn, n), minn)



def main():
    app = QtWidgets.QApplication(sys.argv)  # A new instance of QApplication
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    form = TrainerApp()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    #app.aboutToQuit.connect(form.emit_export_yaml) # myExitHandler is a callable
    app.exec_()  # and execute the app
   
if __name__ == '__main__':  # if we're running file directly and not importing it
    main()
