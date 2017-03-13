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

from time import sleep

# https://github.com/ColinDuquesnoy/QDarkStyleSheet
import qdarkstyle

# https://www.safaribooksonline.com/blog/2014/01/22/create-basic-gui-using-pyqt/
gui = os.path.join(os.path.dirname(__file__), 'look.ui')
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

        # set up command Thread
        self.commandthread = Thread(target = self.commandBuffer)
        self.randomthread = Thread(target=self.randomDriver)
        #self.threadstarted = False
        #self.commandthread.start()

        #flag to stop threads during exit
        #self.running = False
        self.enabled = False
        self.random = False

        self.x = 0.0
        self.y = 0.0
        self.out = 0.0
        self.grasp = 0.0
        self.sliderx = 0.0
        self.slidery = 0.0

        self.previouspoint = Point()
        self.commandpoint =  Point()

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


        rospy.init_node('look', anonymous=True)

        print("INITIALIZED")

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

        self.bus = 0
        self.servo = 0
        self.motorparameter = rospy.ServiceProxy(self.parameterTopic[self.bus], MotorParameter)
        
        self.chkEnable.stateChanged.connect(self.setEnableAll)
        self.chkRandom.stateChanged.connect(self.setRandom)
        self.sliderOut.valueChanged.connect(self.setOut)
        self.sliderGrasp.valueChanged.connect(self.setGrasp)
        self.sliderX.valueChanged.connect(self.setSliderX)
        self.sliderY.valueChanged.connect(self.setSliderY)

        self.pose= [0,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0
                    ]
        for servo in range (0,11):
            self.pose[12 + servo] = CENTERARM[servo]
            self.pose[24 + servo] = CENTERARM[servo]

        print("INIT COMPLETE")  

    def commandBuffer(self):
        while self.enabled == True:
            #self.setgoal(0,3,90)
            #print("triggered!")
            #self.label_15.setText("TRIGGERED!")

            if self.random == True:

                #normalize from 0 to 2
                oldpoint = Point()
                destinationpoint = Point()
                oldpoint.x = self.previouspoint.x
                oldpoint.y = self.previouspoint.y
                destinationpoint.x = self.commandpoint.x
                destinationpoint.y = self.commandpoint.y


                deltax = destinationpoint.x - oldpoint.x
                deltay = destinationpoint.y - oldpoint.y
                
                deltax = clamp(deltax,-0.0015, 0.0015)
                deltay = clamp(deltay,-0.0015, 0.0015)

                self.x = self.previouspoint.x + deltax
                self.y = self.previouspoint.y + deltay

                self.calcValues()

                self.previouspoint.x = self.x
                self.previouspoint.y = self.y

            for bus in range (0,3):
                for servo in range (0,12):
                    self.setGoal(bus,servo,self.pose[(bus * 12) + servo])
            #print(self.pose)
            
            sleep(1.0/HZ)

    def randomDriver(self):
        while self.random == True:
            self.commandpoint.x = random.uniform(-1.0, 1.0)
            self.commandpoint.y = random.uniform(-0.75, 0.75)
            print self.commandpoint
            sleep(random.uniform(3.0, 5.0))

    def mouseMoveEvent(self,event):
        #print("Mouse Cursor Coordinates(x/y):  " + str(event.x()) + "/" + str(event.y()))

        #print( str(self.frameGeometry().width()) + " " + str(self.frameGeometry().height()))

        # grab mouse coord, convert to [-1,1] and clamp
        x = (((float(event.x())/float(self.frame.frameGeometry().width())) * 2) - 1.0)
        x = clamp(x,-1.0,1.0)
        y = (-((float(event.y())/float(self.frame.frameGeometry().height())) * 2) + 1.0)
        y = clamp(y,-1.0,1.0)

        if self.chkFlipX.isChecked() == False:
            x = -x

        if self.chkFlipY.isChecked() == True:
            y = -y

        self.x = x
        self.y = y
        self.out = (float(self.sliderOut.value()) / 100.0)
        self.grasp = (float(self.sliderGrasp.value()) / 100.0)
        self.sliderx = (float(self.sliderX.value()) / 100.0) - 0.5
        self.slidery = (float(self.sliderY.value()) / 100.0) - 0.5

        self.calcValues()


    def calcValues(self):

        x = self.x
        y = self.y
        out = self.out
        grasp = self.grasp
        sliderx = self.sliderx
        slidery = self.slidery

        if self.radioLookAround.isChecked():

            self.pose[0] = clamp(EYERIGHT * x * 2, -EYERIGHT, EYERIGHT)
            self.pose[1] = clamp(EYEUP * y * 2, -EYEUP, EYEUP)
            self.pose[4] = HEADUP * y
            self.pose[3] = HEADRIGHT * x
            self.pose[5] = HEADTILT * y * x

            self.pose[7] = WAISTRIGHT * x

            #if self.chkFlip.isChecked():
            #    waisttilt = -WAISTTILT * y * x
            #else:
            #    waisttilt = WAISTTILT * y * x

            self.pose[6] = clamp(WAISTTILT * (x * y * 2), -WAISTTILT,WAISTTILT)

            #now for the arms...
            for servo in range (0, 12):
                if ( x >= 0 and y < 0):
                    self.pose[12 + servo] = (CENTERARM[servo] + ((OUTSIDEARM[servo] - CENTERARM[servo]) * abs(x* y) ) )
                    self.pose[24 + servo] = (CENTERARM[servo] + ((INSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                if ( x >= 0 and y>= 0):
                    self.pose[24 + servo] = (CENTERARM[servo] + ((OUTSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                    self.pose[12 + servo] = (CENTERARM[servo] + ((INSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                if ( x < 0 and y >= 0):
                    self.pose[12 + servo] = (CENTERARM[servo] + ((OUTSIDEARM[servo] - CENTERARM[servo]) * abs(x* y) ) )
                    self.pose[24 + servo] = (CENTERARM[servo] + ((INSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                if ( x < 0 and y < 0):
                    self.pose[24 + servo] = (CENTERARM[servo] + ((OUTSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                    self.pose[12 + servo] = (CENTERARM[servo] + ((INSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))

        if self.radioLookOut.isChecked():

            self.pose[0] = clamp(EYERIGHT * x * 2, -EYERIGHT, EYERIGHT)
            self.pose[1] = clamp(EYEUP * y * 2, -EYEUP, EYEUP)
            self.pose[4] = HEADUP * y
            self.pose[3] = HEADRIGHT * x
            self.pose[5] = HEADTILT * y * x

            self.pose[7] = WAISTRIGHT * x

            #if self.chkFlip.isChecked():
            #    waisttilt = -WAISTTILT * y * x
            #else:
            #    waisttilt = WAISTTILT * y * x

            self.pose[6] = clamp(-WAISTTILT * (x * y * 2), -WAISTTILT,WAISTTILT)
           
            #now for the arms...
            for servo in range (0, 12):
                if ( x < 0 and y < 0):
                    self.pose[12 + servo] = (CENTERARM[servo] + ((OUTSIDEARM[servo] - CENTERARM[servo]) * abs(x* y) ) )
                    self.pose[24 + servo] = (CENTERARM[servo] + ((INSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                if ( x < 0 and y>= 0):
                    self.pose[24 + servo] = (CENTERARM[servo] + ((OUTSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                    self.pose[12 + servo] = (CENTERARM[servo] + ((INSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                if ( x >= 0 and y >= 0):
                    self.pose[12 + servo] = (CENTERARM[servo] + ((OUTSIDEARM[servo] - CENTERARM[servo]) * abs(x* y) ) )
                    self.pose[24 + servo] = (CENTERARM[servo] + ((INSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                if ( x >= 0 and y < 0):
                    self.pose[24 + servo] = (CENTERARM[servo] + ((OUTSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))
                    self.pose[12 + servo] = (CENTERARM[servo] + ((INSIDEARM[servo] - CENTERARM[servo]) * abs(x * y) ))

        self.label_8.setText("{:10.2f}".format(self.pose[0]))
        self.label_9.setText("{:10.2f}".format(self.pose[1]))
        self.label_10.setText("{:10.2f}".format(self.pose[3]))
        self.label_11.setText("{:10.2f}".format(self.pose[4]))
        self.label_12.setText("{:10.2f}".format(self.pose[5]))
        self.label_13.setText("{:10.2f}".format(self.pose[7]))
        self.label_14.setText("{:10.2f}".format(self.pose[6]))
        #self.label_15.setText("{:10.2f}".format(ARMOUT * y))

        if self.radioPunch.isChecked():

            self.pose[0] = clamp(EYERIGHT * x * 2, -EYERIGHT, EYERIGHT)
            self.pose[1] = clamp(EYEUP * y * 2, -EYEUP, EYEUP)
            self.pose[4] = HEADUP * y
            self.pose[3] = HEADRIGHT * x
            self.pose[5] = HEADTILT * y * x

            self.pose[7] = WAISTRIGHT * x

            #if self.chkFlip.isChecked():
            #    waisttilt = -WAISTTILT * y * x
            #else:
            #    waisttilt = WAISTTILT * y * x

            self.pose[6] = clamp(-WAISTTILT * (x * y * 2), -WAISTTILT,WAISTTILT)
            
            #now for the arms...
            for servo in range (0, 12):
                if ( x < 0 and y < 0):
                    self.pose[12 + servo] = (PUNCHNEUTRAL[servo] + ((PUNCHOUTSIDE[servo] - PUNCHNEUTRAL[servo]) * abs(x* y) ) )
                    self.pose[24 + servo] = (PUNCHNEUTRAL[servo] + ((PUNCHINSIDE[servo] - PUNCHNEUTRAL[servo]) * abs(x * y) ))
                if ( x < 0 and y>= 0):
                    self.pose[24 + servo] = (PUNCHNEUTRAL[servo] + ((PUNCHOUTSIDE[servo] - PUNCHNEUTRAL[servo]) * abs(x * y) ))
                    self.pose[12 + servo] = (PUNCHNEUTRAL[servo] + ((PUNCHINSIDE[servo] - PUNCHNEUTRAL[servo]) * abs(x * y) ))
                if ( x >= 0 and y >= 0):
                    self.pose[12 + servo] = (PUNCHNEUTRAL[servo] + ((PUNCHOUTSIDE[servo] - PUNCHNEUTRAL[servo]) * abs(x* y) ) )
                    self.pose[24 + servo] = (PUNCHNEUTRAL[servo] + ((PUNCHINSIDE[servo] - PUNCHNEUTRAL[servo]) * abs(x * y) ))
                if ( x >= 0 and y < 0):
                    self.pose[24 + servo] = (PUNCHNEUTRAL[servo] + ((PUNCHOUTSIDE[servo] - PUNCHNEUTRAL[servo]) * abs(x * y) ))
                    self.pose[12 + servo] = (PUNCHNEUTRAL[servo] + ((PUNCHINSIDE[servo] - PUNCHNEUTRAL[servo]) * abs(x * y) ))

        if self.radioBeverage.isChecked():
            #out=(float(self.sliderOut.value()) / 100.0)
            #print out
            for servo in range (0, 12):
                self.pose[24 + servo] = (GRABIN[servo] + ((GRABOUT[servo] - GRABIN[servo]) * out))
                self.pose[6] = 0 + int(15.0 * out)  #lean
                self.pose[7] = -30 + int(60.0 * out) + (90 * sliderx) #waist
                self.pose[3] = 40 + int(-97.0 * out) #headleftright
                self.pose[30] = self.pose[30] + (30 * slidery) #bicep

            #fingers
            self.pose[24] = GRABOUT[0] + (90.0 * grasp)
            self.pose[25] = GRABOUT[0] + (90.0 * grasp)
            self.pose[26] = GRABOUT[0] + (90.0 * grasp)
            self.pose[27] = GRABOUT[0] + (90.0 * grasp)

        if self.radioArmOut.isChecked():
            #now for the arms...
            for servo in range (0, 12):
                if x < 0:
                    self.pose[12 + servo] = ( CENTERARM[servo] )
                    self.pose[24 + servo] = (CENTERARM[servo] + ((ARMOUT[servo] - CENTERARM[servo]) * abs(x) ))

                if x >= 0:
                    self.pose[24 + servo] = ( CENTERARM[servo] )
                    self.pose[12 + servo] = (CENTERARM[servo] + ((ARMOUT[servo] - CENTERARM[servo]) * abs(x) ))
        

            self.label_8.setText("{:10.2f}".format(self.pose[0]))
            self.label_9.setText("{:10.2f}".format(self.pose[1])) 
            self.label_10.setText("{:10.2f}".format(self.pose[3]))
            self.label_11.setText("{:10.2f}".format(self.pose[4]))
            self.label_12.setText("{:10.2f}".format(self.pose[5]))
            self.label_13.setText("{:10.2f}".format(self.pose[7]))
            self.label_14.setText("{:10.2f}".format(self.pose[6]))
            #self.label_15.setText("{:10.2f}".format(ARMOUT * y))

    def setOut(self, event):
        self.out = (float(self.sliderOut.value()) / 100.0)
        self.calcValues()

    def setGrasp(self, event):
        self.grasp = (float(self.sliderGrasp.value()) / 100.0)
        self.calcValues()

    def setSliderX(self, event):
        self.sliderx = -((float(self.sliderX.value()) / 100.0) - 0.5) * 2
        self.calcValues()

    def setSliderY(self, event):
        self.slidery = ((float(self.sliderY.value()) / 100.0) - 0.5) * 2
        self.calcValues()
        print self.slidery


    def callback0(self, data):
        if data.id == self.servo and self.bus == 0:
            print data.posraw
            #self.chkEnabled.setChecked(bool(data.enabled))
            #self.txtPosition.setText(str(round(data.position,4)))
            #self.txtSpeed.setText(str(data.presentspeed))
            #self.txtSensorRaw.setText(str(data.posraw))
            #self.chkMoving.setChecked(bool(data.moving))
            #self.chkPower.setChecked(bool(data.power))
            #self.txtGoal.setText(str(data.goal))
  
    def degreestoradians(self, d):
        return d*(3.1415926/180.0)
        
    def sliderChanged(self, i):
        self.txtGoal.setText(str(i/1000.0))
        self.setGoal()

    def setRandom(self):

        self.random = self.chkRandom.isChecked()

        if self.random == True:
            self.randomthread.start()

    def setEnableAll(self):
        for servo in range (0, 12):
            for bus in range (0, 3):
                self.motorcommand.id = servo
                self.motorcommand.parameter = 0x18
                self.motorcommand.value = float(self.chkEnable.isChecked())
                self.commandPublisher[bus].publish(self.motorcommand)
            sleep(0.01)


        self.enabled = self.chkEnable.isChecked()

        if self.enabled == True:
            self.commandthread.start()


        #if self.chkEnable.isChecked() == True:
        #    self.running = True
        #else:
        #    self.running = False

    def setParameter(self, bus, servo, parameter, value):
        rospy.wait_for_service('motorparameter')


    def setGoal(self, bus, servo, goal):
        #print("SETGOAL")
        #print(str(value))

        #goal = float(self.txtGoal.text())

        #self.sliderGoal.setValue(int(goal)*1000.0)

        self.motorcommand.id = servo
        self.motorcommand.parameter = 0x1E
        self.motorcommand.value = goal
        #print(self.motorcommand.value)
        self.commandPublisher[bus].publish(self.motorcommand)
        
        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = [self.jointNames[((bus * 12) + servo)]]
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
        self.sliderGoal.setValue(int(value * 1000.0))
       
    def getMinGoal(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        value = self.motorparameter(self.cmbServo.currentIndex(), 0x06).data
        self.txtMinGoal.setText(str(value))
        self.sliderGoal.setMinimum(int(value * 1000.0))
    
       
    def getMaxGoal(self):
        #bus = self.cmbBus.currentIndex()
        #motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        rospy.wait_for_service('/servobus/torso/motorparameter')
        value = self.motorparameter(self.cmbServo.currentIndex(), 0x08).data
        self.txtMaxGoal.setText(str(value))
        self.sliderGoal.setMaximum(int(value * 1000.0))
        
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

    def closeEvent(self, event):
        self.enabled = False
        self.random = False
        print "GOODBYE!"

class Point:
    x = 0.0
    y = 0.0

def clamp(n,minn,maxn):
    return max(min(maxn, n), minn)

HZ          =  40

EYERIGHT    =  15 #overcommand for effect
EYEUP       =  15 #overcommand for effect
HEADRIGHT   =  60
HEADUP      = -20
HEADTILT    = -15
WAISTRIGHT  =  30
WAISTTILT   =  15  
ARMOUT      =  15

CENTERARM = [
             60,    #pinky
             60,    #ring
             80,    #middle
             60,    #index
             45,    #thumb
             45,    #hand
             40,    #bicep
            -20,    #bicep_rotate
              5,    #shoulder_side
            -20,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
]

OUTSIDEARM = [
             24,    #pinky
             12,    #ring
             21,    #middle
             10,    #index
             10,    #thumb
             90,    #hand
             21,    #bicep
              0,    #bicep_rotate
             15,    #shoulder_side
            -15,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]

INSIDEARM = [
             60,    #pinky
             60,    #ring
             80,    #middle
             60,    #index
             45,    #thumb
             45,    #hand
             30,    #bicep
            -37,    #bicep_rotate
             10,    #shoulder_side
            -15,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]

PUNCHOUTSIDE = [
             55,    #pinky
             60,    #ring
             80,    #middle
             60,    #index
             45,    #thumb
             45,    #hand
             05,    #bicep
            -21,    #bicep_rotate
             23,    #shoulder_side
             76,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]

PUNCHINSIDE = [
             52,    #pinky
             71,    #ring
             85,    #middle
             66,    #index
            109,    #thumb
             48,    #hand
             85,    #bicep
            -18,    #bicep_rotate
             20,    #shoulder_side
            -45,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]

PUNCHNEUTRAL = [
             52,    #pinky
             71,    #ring
             85,    #middle
             66,    #index
            109,    #thumb
             64,    #hand
             85,    #bicep
            -8,    #bicep_rotate
              5,    #shoulder_side
            -45,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]
		
GRABIN = [
             12,    #pinky
              8,    #ring
              7,    #middle
             17,    #index
              7,    #thumb
            100,    #hand
             95,    #bicep
            -50,    #bicep_rotate
              5,    #shoulder_side
             20,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]
		
GRABOUT = [
             12,    #pinky
              8,    #ring
              7,    #middle
             17,    #index
              7,    #thumb
             93,    #hand
             65,    #bicep
             48,    #bicep_rotate
             35,    #shoulder_side
             20,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]
		
GRABNEUTRAL = [
             52,    #pinky
             71,    #ring
             85,    #middle
             66,    #index
            109,    #thumb
             64,    #hand
             85,    #bicep
             -8,    #bicep_rotate
              5,    #shoulder_side
            -45,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]

ARMOUT = [
             10,    #pinky
             12,    #ring
             13,    #middle
             21,    #index
             12,    #thumb
            105,    #hand
             29,    #bicep
             54,    #bicep_rotate
             50,    #shoulder_side
              0,    #shoulder_up
             00,    #arm-nc-10
             00     #arm-nc-11
        ]

def main():
    app = QtWidgets.QApplication(sys.argv)  # A new instance of QApplication
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    form = ExampleApp()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app
   
if __name__ == '__main__':  # if we're running file directly and not importing it
    main()

