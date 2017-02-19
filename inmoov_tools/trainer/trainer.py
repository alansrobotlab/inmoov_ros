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

# https://github.com/ColinDuquesnoy/QDarkStyleSheet
import qdarkstyle

#Ui_MainWindow, QtBaseClass = uic.loadUiType("trainer.ui")
# https://www.safaribooksonline.com/blog/2014/01/22/create-basic-gui-using-pyqt/
gui = os.path.join(os.path.dirname(__file__), 'trainer.ui')
Ui_MainWindow, QtBaseClass = uic.loadUiType(gui)


# https://nikolak.com/pyqt-qt-designer-getting-started/
class ExampleApp(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        # Explaining super is out of the scope of this article
        # So please google it if you're not familar with it
        # Simple reason why we use it here is that it allows us to
        # access variables, methods etc in the design.py file
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
        # It sets up layout and widgets that are defined

        self.parameterTopic = ["servobus/torso/motorparameter","servobus/leftarm/motorparameter","servobus/rightarm/motorparameter"]

        self.motorcommand = MotorCommand()
        self.jointcommand = JointState()
        
        self.jointNames = []
        
        for servo in range (0, 12):
            self.jointNames.append( rospy.get_param('servobus/torso/servomap/'+str(servo)+'/name'))
            
        for servo in range (0, 12):
            self.jointNames.append( rospy.get_param('servobus/leftarm/servomap/'+str(servo)+'/name'))
            
        for servo in range (0, 12):
            self.jointNames.append( rospy.get_param('servobus/rightarm/servomap/'+str(servo)+'/name'))
        
        print(self.jointNames)
        
        
            #'right_pinky','right_ring','right_middle','right_index','right_thumb',
            #'right_hand','right_bicep','right_bicep_rotate','right_shoulder_side','right_shoulder_up','','',
            
            #'eye_leftright','eyes_updown','jaw','head_leftright','head_updown','head_tilt','waist_lean','waist_rotate','','','','',  
            
            #'left_pinky','left_ring','left_middle','left_index','left_thumb',
            #'left_hand','left_bicep','left_bicep_rotate','left_shoulder_side','left_shoulder_up','','',
            


        self.setupDropDowns()
        




        rospy.init_node('trainer', anonymous=True)

        print("INITIALIZED")
        
        self.commandPublisher = []
        self.commandPublisher.append(rospy.Publisher("servobus/torso/motorcommand", MotorCommand, queue_size=10))
        self.commandPublisher.append(rospy.Publisher("servobus/leftarm/motorcommand", MotorCommand, queue_size=10))
        self.commandPublisher.append(rospy.Publisher("servobus/rightarm/motorcommand", MotorCommand, queue_size=10))
        
        print("COMMANDS COMPLETE")

        self.statusSubscriber = []
        self.statusSubscriber.append(rospy.Subscriber("servobus/torso/motorstatus", MotorStatus, self.callback0))
        self.statusSubscriber.append(rospy.Subscriber("servobus/leftarm/motorstatus", MotorStatus, self.callback1))
        self.statusSubscriber.append(rospy.Subscriber("servobus/rightarm/motorstatus", MotorStatus, self.callback2))
        
        print("SUBSCRIBER COMPLETE")

        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)
        
        print("JOINTPUBLISHER COMPLETE")

        self.bus = 0
        self.servo = 0
        self.motorparameter = rospy.ServiceProxy(self.parameterTopic[self.bus], MotorParameter)
        
        self.busChanged()

        self.servoChanged()

        self.cmbBus.currentIndexChanged.connect(self.busChanged)
        self.cmbServo.currentIndexChanged.connect(self.servoChanged)

        self.txtGoal.editingFinished.connect(self.setGoal)
        self.txtMinPulse.editingFinished.connect(self.setMinPulse)
        self.txtMaxPulse.editingFinished.connect(self.setMaxPulse)
        self.txtMinGoal.editingFinished.connect(self.setMinGoal)
        self.txtMaxGoal.editingFinished.connect(self.setMaxGoal)
        self.txtMinSensor.editingFinished.connect(self.setMinSensor)
        self.txtMaxSensor.editingFinished.connect(self.setMaxSensor)
        self.chkEnabled.stateChanged.connect(self.setEnabled)
        self.chkCalibrated.stateChanged.connect(self.setCalibrated)
        
        self.sliderGoal.valueChanged.connect(self.sliderChanged)

        self.chkEnableAll.stateChanged.connect(self.setEnableAll)
        
        print("INIT COMPLETE")

    def busChanged(self):
        
        print("BUSCHANGED")

        self.bus = self.cmbBus.currentIndex()
        self.motorparameter = rospy.ServiceProxy(self.parameterTopic[self.bus], MotorParameter)
        
        #self.cmbServo.currentIndexChanged.disconnect(self.servoChanged)

        self.cmbServo.clear()
        
        for s in range(0, 11):
            self.cmbServo.addItem(self.jointNames[(self.bus * 12) + s])

        #self.cmbServo.currentIndexChanged.connect(self.servoChanged)

        #self.servoChanged()

    def servoChanged(self):
    
        print("SERVOCHANGED")

        if self.cmbServo.count() > 0:
            print("SERVICECALLS")
            self.servo = self.cmbServo.currentIndex()
            
            sdir = 0.1

            self.getMinPulse()
            #print("MINPULSE")
            sleep(sdir)
            self.getMaxPulse()
            #print("MAXPULSE")
            sleep(sdir)     
            self.getMinMaxGoal()       
            #self.getMinGoal()
            #print("MINGOAL")
            #sleep(sdir)
            #self.getMaxGoal()
            #print("MAXGOAL")
            sleep(sdir)
            self.getGoal()
            #print("GOAL")
            sleep(sdir)
            self.getMinSensor()
            #print("MINSENSOR")
            sleep(sdir)
            self.getMaxSensor()
            #print("MAXSENSOR")
            sleep(sdir)
            self.getEnabled()
            #print("ENABLED")
            sleep(sdir)
            self.getCalibrated()
            #print("CALIBRATED")
            sleep(sdir)
    

    def callback0(self, data):
        if data.id == self.servo and self.bus == 0:
            #print data.posraw
            #self.chkEnabled.setChecked(bool(data.enabled))
            self.txtPosition.setText(str(round(data.position,4)))
            self.txtSpeed.setText(str(data.presentspeed))
            self.txtSensorRaw.setText(str(data.posraw))
            self.chkMoving.setChecked(bool(data.moving))
            self.chkPower.setChecked(bool(data.power))
            #self.txtGoal.setText(str(data.goal))
            
    def callback1(self, data):
        if data.id == self.servo and self.bus == 1:
            #print data.posraw
            #self.chkEnabled.setChecked(bool(data.enabled))
            self.txtPosition.setText(str(round(data.position,4)))
            self.txtSpeed.setText(str(data.presentspeed))
            self.txtSensorRaw.setText(str(data.posraw))
            self.chkMoving.setChecked(bool(data.moving))
            self.chkPower.setChecked(bool(data.power))
            #self.txtGoal.setText(str(data.goal))

    def callback2(self, data):
        if data.id == self.servo and self.bus == 2:
            #print data.posraw
            #self.chkEnabled.setChecked(bool(data.enabled))
            self.txtPosition.setText(str(round(data.position,4)))
            self.txtSpeed.setText(str(data.presentspeed))
            self.txtSensorRaw.setText(str(data.posraw))
            self.chkMoving.setChecked(bool(data.moving))
            self.chkPower.setChecked(bool(data.power))
            #self.txtGoal.setText(str(data.goal))
    
    def degreestoradians(self, d):
        return d*(3.1415926/180.0)
    
    def setupDropDowns(self):
    
        self.cmbBus.addItem(rospy.get_param('/servobus/torso/name'))
        self.cmbBus.addItem(rospy.get_param('/servobus/leftarm/name'))
        self.cmbBus.addItem(rospy.get_param('/servobus/rightarm/name'))

        for servo in range (0, 11):
            #print('/servobus/torso/servomap/' + str(servo) + '/name')
            self.cmbServo.addItem(rospy.get_param('/servobus/torso/servomap/' + str(servo) + '/name'))

        self.cmbSmoothing.addItem('0 - Instant')
        self.cmbSmoothing.addItem('1 - Max Speed')
        self.cmbSmoothing.addItem('2 - Linear Ramp')
        self.cmbSmoothing.addItem('3 - COS Ramp')
        self.cmbSmoothing.addItem('4 - COS^2 Ramp')
        
    def sliderChanged(self, i):
        self.txtGoal.setText(str(i/1000.0))
        self.setGoal()
    
    def setEnableAll(self):
        for servo in range (0, 12):
            for bus in range (0, 3):
                self.motorcommand.id = servo
                self.motorcommand.parameter = 0x18
                self.motorcommand.value = float(self.chkEnableAll.isChecked())
                self.commandPublisher[bus].publish(self.motorcommand)
            #sleep(0.1)

    def setParameter(self, bus, servo, parameter, value):
        rospy.wait_for_service('motorparameter')


    def setGoal(self):
        print("SETGOAL")
        #print(str(value))

        goal = float(self.txtGoal.text())

        self.sliderGoal.setValue(int(goal)*1000.0)

        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x1E
        self.motorcommand.value = goal
        #print(self.motorcommand.value)
        self.commandPublisher[self.bus].publish(self.motorcommand)
        
        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = [self.jointNames[((self.bus * 12) + self.servo)]]
        self.jointcommand.position = [self.degreestoradians(goal)]
        self.jointcommand.velocity = []
        self.jointcommand.effort = []
        self.jointPublisher.publish(self.jointcommand)
        
    
    def getGoal(self):
        print("GETGOAL")
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        value = self.motorparameter(self.cmbServo.currentIndex(), 0x1E).data
        self.txtGoal.setText(str(value))
        val = clamp(int(value * 1000.00),-360000,360000)
        self.sliderGoal.setValue(val)
        #self.sliderGoal.setValue(int(value * 1000.0))
       
    def setMinPulse(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x14
        self.motorcommand.value = float(self.txtMinPulse.text())
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMinPulse(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        self.txtMinPulse.setText(str(self.motorparameter(self.cmbServo.currentIndex(), 0x14).data))
    
    def setMaxPulse(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x16
        self.motorcommand.value = float(self.txtMaxPulse.text())
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMaxPulse(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        self.txtMaxPulse.setText(str(self.motorparameter(self.cmbServo.currentIndex(), 0x16).data))
        
    def setMinGoal(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x06
        self.motorcommand.value = float(self.txtMinGoal.text())
        self.sliderGoal.setMinimum(int(self.motorcommand.value * 1000.0))
        self.commandPublisher[self.bus].publish(self.motorcommand)

    def getMinMaxGoal(self):
        rospy.wait_for_service('/servobus/torso/motorparameter')
        minval = self.motorparameter(self.cmbServo.currentIndex(), 0x06).data
        self.txtMinGoal.setText(str(minval))
        
        rospy.wait_for_service('/servobus/torso/motorparameter')
        maxval = self.motorparameter(self.cmbServo.currentIndex(), 0x08).data
        self.txtMaxGoal.setText(str(maxval))
        if minval < maxval :
            self.sliderGoal.setMinimum(int(minval * 1000.0))
            self.sliderGoal.setMaximum(int(maxval * 1000.0))
        else:
            self.sliderGoal.setMinimum(int(maxval * 1000.0))
            self.sliderGoal.setMaximum(int(minval * 1000.0))
       
    def getMinGoal(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        value = self.motorparameter(self.cmbServo.currentIndex(), 0x06).data
        self.txtMinGoal.setText(str(value))
        self.sliderGoal.setMinimum(int(value * 1000.0))
    
    def setMaxGoal(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x08
        self.motorcommand.value = float(self.txtMaxGoal.text())
        self.sliderGoal.setMaximum(int(self.motorcommand.value * 1000.0))
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMaxGoal(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        value = self.motorparameter(self.cmbServo.currentIndex(), 0x08).data
        self.txtMaxGoal.setText(str(value))
        self.sliderGoal.setMaximum(int(value * 1000.0))
        
    def setMinSensor(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0xA2
        self.motorcommand.value = float(self.txtMinSensor.text())
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMinSensor(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        self.txtMinSensor.setText(str(self.motorparameter(self.cmbServo.currentIndex(), 0xA2).data))
    
    def setMaxSensor(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0xA4
        self.motorcommand.value = float(self.txtMaxSensor.text())
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMaxSensor(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        self.txtMaxSensor.setText(str(self.motorparameter(self.cmbServo.currentIndex(), 0xA4).data))
        
    def setEnabled(self):
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x18
        self.motorcommand.value = float(self.chkEnabled.isChecked())
        self.commandPublisher[self.bus].publish(self.motorcommand)
    
    def getEnabled(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        self.chkEnabled.setChecked(bool(self.motorparameter(self.cmbServo.currentIndex(), 0x18).data))
        
    def setCalibrated(self):
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0xA0
        self.motorcommand.value = float(self.chkCalibrated.isChecked())
        self.commandPublisher[self.bus].publish(self.motorcommand)
    
    def getCalibrated(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        self.chkCalibrated.setChecked(bool(self.motorparameter(self.cmbServo.currentIndex(), 0xA0).data))

def clamp(n,minn,maxn):
    return max(min(maxn, n), minn)

def main():
    app = QtWidgets.QApplication(sys.argv)  # A new instance of QApplication
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    #sshFile="/home/grey/inmoov-grey/src/inmoov_tools/trainer/darkorange.stylesheet"
    #with open(sshFile,"r") as fh:
    #    app.setStyleSheet(fh.read())
    form = ExampleApp()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app
   
if __name__ == '__main__':  # if we're running file directly and not importing it
    main()
