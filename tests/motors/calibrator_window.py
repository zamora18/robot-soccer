# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'calibratorwindow.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
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
        MainWindow.resize(579, 424)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName(_fromUtf8("centralWidget"))
        self.M1PWMSlider = QtGui.QSlider(self.centralWidget)
        self.M1PWMSlider.setGeometry(QtCore.QRect(60, 80, 431, 22))
        self.M1PWMSlider.setMaximum(127)
        self.M1PWMSlider.setOrientation(QtCore.Qt.Horizontal)
        self.M1PWMSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.M1PWMSlider.setObjectName(_fromUtf8("M1PWMSlider"))
        self.label = QtGui.QLabel(self.centralWidget)
        self.label.setGeometry(QtCore.QRect(30, 80, 21, 20))
        self.label.setObjectName(_fromUtf8("label"))
        self.startButton = QtGui.QPushButton(self.centralWidget)
        self.startButton.setGeometry(QtCore.QRect(140, 230, 113, 32))
        self.startButton.setObjectName(_fromUtf8("startButton"))
        self.stopButton = QtGui.QPushButton(self.centralWidget)
        self.stopButton.setGeometry(QtCore.QRect(270, 230, 113, 32))
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.label_2 = QtGui.QLabel(self.centralWidget)
        self.label_2.setGeometry(QtCore.QRect(30, 130, 21, 20))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.M2PWMSlider = QtGui.QSlider(self.centralWidget)
        self.M2PWMSlider.setGeometry(QtCore.QRect(60, 130, 431, 22))
        self.M2PWMSlider.setMaximum(127)
        self.M2PWMSlider.setOrientation(QtCore.Qt.Horizontal)
        self.M2PWMSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.M2PWMSlider.setObjectName(_fromUtf8("M2PWMSlider"))
        self.label_3 = QtGui.QLabel(self.centralWidget)
        self.label_3.setGeometry(QtCore.QRect(30, 180, 21, 20))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.M3PWMSlider = QtGui.QSlider(self.centralWidget)
        self.M3PWMSlider.setGeometry(QtCore.QRect(60, 180, 431, 22))
        self.M3PWMSlider.setMaximum(127)
        self.M3PWMSlider.setOrientation(QtCore.Qt.Horizontal)
        self.M3PWMSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.M3PWMSlider.setTickInterval(10)
        self.M3PWMSlider.setObjectName(_fromUtf8("M3PWMSlider"))
        self.m1PWM = QtGui.QLabel(self.centralWidget)
        self.m1PWM.setGeometry(QtCore.QRect(510, 80, 59, 20))
        self.m1PWM.setObjectName(_fromUtf8("m1PWM"))
        self.m2PWM = QtGui.QLabel(self.centralWidget)
        self.m2PWM.setGeometry(QtCore.QRect(510, 130, 59, 20))
        self.m2PWM.setObjectName(_fromUtf8("m2PWM"))
        self.m3PWM = QtGui.QLabel(self.centralWidget)
        self.m3PWM.setGeometry(QtCore.QRect(510, 180, 59, 20))
        self.m3PWM.setObjectName(_fromUtf8("m3PWM"))
        self.label_4 = QtGui.QLabel(self.centralWidget)
        self.label_4.setGeometry(QtCore.QRect(30, 40, 101, 16))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.label_5 = QtGui.QLabel(self.centralWidget)
        self.label_5.setGeometry(QtCore.QRect(270, 41, 101, 16))
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.txtDriveTime = QtGui.QTextEdit(self.centralWidget)
        self.txtDriveTime.setGeometry(QtCore.QRect(140, 40, 101, 21))
        self.txtDriveTime.setObjectName(_fromUtf8("txtDriveTime"))
        self.txtSleepTime = QtGui.QTextEdit(self.centralWidget)
        self.txtSleepTime.setGeometry(QtCore.QRect(380, 40, 101, 21))
        self.txtSleepTime.setObjectName(_fromUtf8("txtSleepTime"))
        self.saveButton = QtGui.QPushButton(self.centralWidget)
        self.saveButton.setGeometry(QtCore.QRect(490, 36, 51, 32))
        self.saveButton.setObjectName(_fromUtf8("saveButton"))
        MainWindow.setCentralWidget(self.centralWidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.label.setText(_translate("MainWindow", "M1", None))
        self.startButton.setText(_translate("MainWindow", "Start", None))
        self.stopButton.setText(_translate("MainWindow", "Stop", None))
        self.label_2.setText(_translate("MainWindow", "M2", None))
        self.label_3.setText(_translate("MainWindow", "M3", None))
        self.m1PWM.setText(_translate("MainWindow", "M1", None))
        self.m2PWM.setText(_translate("MainWindow", "M2", None))
        self.m3PWM.setText(_translate("MainWindow", "M3", None))
        self.label_4.setText(_translate("MainWindow", "Drive Time (ms):", None))
        self.label_5.setText(_translate("MainWindow", "Sleep Time (ms):", None))
        self.saveButton.setText(_translate("MainWindow", "Save", None))

