# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'trainergui.ui'
#
# Created: Tue May 24 14:29:31 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(401, 686)
        MainWindow.setAutoFillBackground(False)
        MainWindow.setDocumentMode(False)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.sliderGoal = QtGui.QSlider(self.centralwidget)
        self.sliderGoal.setGeometry(QtCore.QRect(340, 20, 31, 611))
        self.sliderGoal.setMaximum(990000)
        self.sliderGoal.setSingleStep(10000)
        self.sliderGoal.setPageStep(100000)
        self.sliderGoal.setOrientation(QtCore.Qt.Vertical)
        self.sliderGoal.setInvertedAppearance(False)
        self.sliderGoal.setInvertedControls(False)
        self.sliderGoal.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.sliderGoal.setTickInterval(10000)
        self.sliderGoal.setObjectName(_fromUtf8("sliderGoal"))
        self.frame = QtGui.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(30, 20, 281, 611))
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName(_fromUtf8("frame"))
        self.txtMinPulse = QtGui.QLineEdit(self.frame)
        self.txtMinPulse.setGeometry(QtCore.QRect(110, 310, 141, 27))
        self.txtMinPulse.setObjectName(_fromUtf8("txtMinPulse"))
        self.txtMaxSensor = QtGui.QLineEdit(self.frame)
        self.txtMaxSensor.setGeometry(QtCore.QRect(110, 400, 141, 27))
        self.txtMaxSensor.setObjectName(_fromUtf8("txtMaxSensor"))
        self.label_3 = QtGui.QLabel(self.frame)
        self.label_3.setGeometry(QtCore.QRect(16, 310, 91, 21))
        self.label_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.frame)
        self.label_4.setGeometry(QtCore.QRect(16, 340, 91, 21))
        self.label_4.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.label_5 = QtGui.QLabel(self.frame)
        self.label_5.setGeometry(QtCore.QRect(16, 370, 91, 21))
        self.label_5.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.label_2 = QtGui.QLabel(self.frame)
        self.label_2.setGeometry(QtCore.QRect(40, 280, 67, 21))
        self.label_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.txtMaxPulse = QtGui.QLineEdit(self.frame)
        self.txtMaxPulse.setGeometry(QtCore.QRect(110, 340, 141, 27))
        self.txtMaxPulse.setObjectName(_fromUtf8("txtMaxPulse"))
        self.chkCalibrated = QtGui.QCheckBox(self.frame)
        self.chkCalibrated.setGeometry(QtCore.QRect(30, 430, 97, 22))
        self.chkCalibrated.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.chkCalibrated.setObjectName(_fromUtf8("chkCalibrated"))
        self.txtMaxGoal = QtGui.QLineEdit(self.frame)
        self.txtMaxGoal.setGeometry(QtCore.QRect(110, 280, 141, 27))
        self.txtMaxGoal.setObjectName(_fromUtf8("txtMaxGoal"))
        self.txtMinSensor = QtGui.QLineEdit(self.frame)
        self.txtMinSensor.setGeometry(QtCore.QRect(110, 370, 141, 27))
        self.txtMinSensor.setObjectName(_fromUtf8("txtMinSensor"))
        self.label = QtGui.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(40, 250, 67, 21))
        self.label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label.setObjectName(_fromUtf8("label"))
        self.cmbSmoothing = QtGui.QComboBox(self.frame)
        self.cmbSmoothing.setEnabled(False)
        self.cmbSmoothing.setGeometry(QtCore.QRect(110, 200, 141, 27))
        self.cmbSmoothing.setObjectName(_fromUtf8("cmbSmoothing"))
        self.label_8 = QtGui.QLabel(self.frame)
        self.label_8.setEnabled(False)
        self.label_8.setGeometry(QtCore.QRect(16, 170, 91, 21))
        self.label_8.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.txtMaxSpeed = QtGui.QLineEdit(self.frame)
        self.txtMaxSpeed.setEnabled(False)
        self.txtMaxSpeed.setGeometry(QtCore.QRect(110, 170, 141, 27))
        self.txtMaxSpeed.setObjectName(_fromUtf8("txtMaxSpeed"))
        self.label_6 = QtGui.QLabel(self.frame)
        self.label_6.setGeometry(QtCore.QRect(16, 400, 91, 21))
        self.label_6.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_6.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.label_14 = QtGui.QLabel(self.frame)
        self.label_14.setEnabled(False)
        self.label_14.setGeometry(QtCore.QRect(10, 200, 91, 21))
        self.label_14.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.txtMinGoal = QtGui.QLineEdit(self.frame)
        self.txtMinGoal.setGeometry(QtCore.QRect(110, 250, 141, 27))
        self.txtMinGoal.setObjectName(_fromUtf8("txtMinGoal"))
        self.label_7 = QtGui.QLabel(self.frame)
        self.label_7.setGeometry(QtCore.QRect(18, 560, 91, 21))
        self.label_7.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.txtSpeed = QtGui.QLineEdit(self.frame)
        self.txtSpeed.setEnabled(False)
        self.txtSpeed.setGeometry(QtCore.QRect(132, 520, 121, 27))
        self.txtSpeed.setObjectName(_fromUtf8("txtSpeed"))
        self.chkPower = QtGui.QCheckBox(self.frame)
        self.chkPower.setEnabled(False)
        self.chkPower.setGeometry(QtCore.QRect(30, 500, 97, 22))
        self.chkPower.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.chkPower.setText(_fromUtf8(""))
        self.chkPower.setObjectName(_fromUtf8("chkPower"))
        self.txtPosition = QtGui.QLineEdit(self.frame)
        self.txtPosition.setEnabled(False)
        self.txtPosition.setGeometry(QtCore.QRect(110, 470, 141, 27))
        self.txtPosition.setObjectName(_fromUtf8("txtPosition"))
        self.txtSensorRaw = QtGui.QLineEdit(self.frame)
        self.txtSensorRaw.setEnabled(False)
        self.txtSensorRaw.setGeometry(QtCore.QRect(112, 560, 141, 27))
        self.txtSensorRaw.setObjectName(_fromUtf8("txtSensorRaw"))
        self.label_10 = QtGui.QLabel(self.frame)
        self.label_10.setGeometry(QtCore.QRect(40, 470, 67, 21))
        self.label_10.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.label_11 = QtGui.QLabel(self.frame)
        self.label_11.setGeometry(QtCore.QRect(10, 500, 91, 17))
        self.label_11.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.cmbServo = QtGui.QComboBox(self.frame)
        self.cmbServo.setGeometry(QtCore.QRect(110, 50, 141, 27))
        self.cmbServo.setObjectName(_fromUtf8("cmbServo"))
        self.label_12 = QtGui.QLabel(self.frame)
        self.label_12.setGeometry(QtCore.QRect(30, 20, 71, 21))
        self.label_12.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.chkEnabled = QtGui.QCheckBox(self.frame)
        self.chkEnabled.setGeometry(QtCore.QRect(30, 100, 97, 22))
        self.chkEnabled.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.chkEnabled.setObjectName(_fromUtf8("chkEnabled"))
        self.cmbBus = QtGui.QComboBox(self.frame)
        self.cmbBus.setGeometry(QtCore.QRect(110, 20, 141, 27))
        self.cmbBus.setFrame(True)
        self.cmbBus.setObjectName(_fromUtf8("cmbBus"))
        self.label_13 = QtGui.QLabel(self.frame)
        self.label_13.setGeometry(QtCore.QRect(30, 50, 71, 21))
        self.label_13.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.label_9 = QtGui.QLabel(self.frame)
        self.label_9.setGeometry(QtCore.QRect(40, 130, 67, 21))
        self.label_9.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.txtGoal = QtGui.QLineEdit(self.frame)
        self.txtGoal.setGeometry(QtCore.QRect(110, 130, 141, 27))
        self.txtGoal.setObjectName(_fromUtf8("txtGoal"))
        self.chkMoving = QtGui.QCheckBox(self.frame)
        self.chkMoving.setEnabled(False)
        self.chkMoving.setGeometry(QtCore.QRect(30, 520, 97, 22))
        self.chkMoving.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.chkMoving.setText(_fromUtf8(""))
        self.chkMoving.setObjectName(_fromUtf8("chkMoving"))
        self.label_15 = QtGui.QLabel(self.frame)
        self.label_15.setGeometry(QtCore.QRect(10, 520, 91, 17))
        self.label_15.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setSizeGripEnabled(False)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        MainWindow.setTabOrder(self.cmbBus, self.cmbServo)
        MainWindow.setTabOrder(self.cmbServo, self.chkEnabled)
        MainWindow.setTabOrder(self.chkEnabled, self.txtGoal)
        MainWindow.setTabOrder(self.txtGoal, self.txtMaxSpeed)
        MainWindow.setTabOrder(self.txtMaxSpeed, self.cmbSmoothing)
        MainWindow.setTabOrder(self.cmbSmoothing, self.txtMinGoal)
        MainWindow.setTabOrder(self.txtMinGoal, self.txtMaxGoal)
        MainWindow.setTabOrder(self.txtMaxGoal, self.txtMinPulse)
        MainWindow.setTabOrder(self.txtMinPulse, self.txtMaxPulse)
        MainWindow.setTabOrder(self.txtMaxPulse, self.txtMinSensor)
        MainWindow.setTabOrder(self.txtMinSensor, self.txtMaxSensor)
        MainWindow.setTabOrder(self.txtMaxSensor, self.chkCalibrated)
        MainWindow.setTabOrder(self.chkCalibrated, self.txtPosition)
        MainWindow.setTabOrder(self.txtPosition, self.chkPower)
        MainWindow.setTabOrder(self.chkPower, self.chkMoving)
        MainWindow.setTabOrder(self.chkMoving, self.txtSpeed)
        MainWindow.setTabOrder(self.txtSpeed, self.txtSensorRaw)
        MainWindow.setTabOrder(self.txtSensorRaw, self.sliderGoal)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "ROS Servo Trainer", None))
        self.label_3.setText(_translate("MainWindow", "Min Pulse", None))
        self.label_4.setText(_translate("MainWindow", "Max Pulse", None))
        self.label_5.setText(_translate("MainWindow", "MinSensor", None))
        self.label_2.setText(_translate("MainWindow", "Max Goal", None))
        self.chkCalibrated.setText(_translate("MainWindow", "Calibrated", None))
        self.label.setText(_translate("MainWindow", "Min Goal", None))
        self.label_8.setText(_translate("MainWindow", "MaxSpeed", None))
        self.label_6.setText(_translate("MainWindow", "Max Sensor", None))
        self.label_14.setText(_translate("MainWindow", "Smoothing", None))
        self.label_7.setText(_translate("MainWindow", "Sensor Raw", None))
        self.label_10.setText(_translate("MainWindow", "Position", None))
        self.label_11.setText(_translate("MainWindow", "Power", None))
        self.label_12.setText(_translate("MainWindow", "Bus", None))
        self.chkEnabled.setText(_translate("MainWindow", "Enabled", None))
        self.label_13.setText(_translate("MainWindow", "Servo", None))
        self.label_9.setText(_translate("MainWindow", "Goal", None))
        self.label_15.setText(_translate("MainWindow", "Moving", None))
