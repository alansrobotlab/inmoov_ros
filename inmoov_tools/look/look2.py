#!/usr/bin/env python

import sys
import os
import rospy
import rospkg


from threading import Thread
import thread
import atexit

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding.QtGui import QWidget

from PyQt4 import QtGui, QtCore, uic

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
gui = os.path.join(os.path.dirname(__file__), 'lookgui.ui')
form_class = uic.loadUiType(gui)[0]

# https://nikolak.com/pyqt-qt-designer-getting-started/
#class ExampleApp(QtGui.QMainWindow, Ui_MainWindow):
class ExampleApp(QtGui.QMainWindow, form_class):
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
        self.threadstarted = False
        #self.commandthread.start()

        #flag to stop threads during exit
        self.running = False

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

        self.pose= [0,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,0,0
                    ]
        for servo in range (0,11):
            self.pose[12 + servo] = CENTERARM[servo]
            self.pose[24 + servo] = CENTERARM[servo]

        print("INIT COMPLETE")  

    def commandBuffer(self):
        while self.threadstarted == True:
            while self.running == True:
                #self.setgoal(0,3,90)
                #print("triggered!")
                self.label_15.setText("TRIGGERED!")
                for bus in range (0,3):
                    for servo in range (0,12):
                        self.setGoal(bus,servo,self.pose[(bus * 12) + servo])
                #print(self.pose)
            sleep(1.0/HZ)

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

        self.calcValues(x,y)


    def calcValues(self,x,y):

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

        # self.label_8.setText("{:10.2f}".format(eyeright))
        #self.label_8.setText(str(EYERIGHT * x))
        #self.label_9.setText("{:10.2f}".format(eyeup))
        #self.label_10.setText("{:10.2f}".format(headright))
        #self.label_11.setText("{:10.2f}".format(headup))
        #self.label_12.setText("{:10.2f}".format(headtilt))
        #self.label_13.setText("{:10.2f}".format(waistright))
        #self.label_14.setText("{:10.2f}".format(waisttilt))
        #self.label_15.setText("{:10.2f}".format(ARMOUT * y))




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
    
    def setEnableAll(self):

        for servo in range (0, 11):
            for bus in range (1, 3):
                self.setGoal(bus,servo,CENTERARM[servo])

    def setEnableAll(self):
        for servo in range (0, 12):
            for bus in range (0, 3):
                self.motorcommand.id = servo
                self.motorcommand.parameter = 0x18
                self.motorcommand.value = float(self.chkEnable.isChecked())
                self.commandPublisher[bus].publish(self.motorcommand)
            sleep(0.01)


        self.running = self.chkEnable.isChecked()

        if self.threadstarted == False:
            self.threadstarted = True
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
        self.running = False
        self.threadstarted = False
        print "GOODBYE!"




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

def main():
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    app.setStyleSheet(qdarkstyle.load_stylesheet(pyside=False))
    form = ExampleApp()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app
   
if __name__ == '__main__':  # if we're running file directly and not importing it
    main()
