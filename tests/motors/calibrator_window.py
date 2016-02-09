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
        MainWindow.resize(510, 424)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName(_fromUtf8("centralWidget"))
        self.M1PWMSlider = QtGui.QSlider(self.centralWidget)
        self.M1PWMSlider.setGeometry(QtCore.QRect(60, 80, 361, 22))
        self.M1PWMSlider.setMaximum(127)
        self.M1PWMSlider.setOrientation(QtCore.Qt.Horizontal)
        self.M1PWMSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.M1PWMSlider.setObjectName(_fromUtf8("M1PWMSlider"))
        self.label = QtGui.QLabel(self.centralWidget)
        self.label.setGeometry(QtCore.QRect(430, 80, 59, 20))
        self.label.setObjectName(_fromUtf8("label"))
        self.startButton = QtGui.QPushButton(self.centralWidget)
        self.startButton.setGeometry(QtCore.QRect(130, 270, 113, 32))
        self.startButton.setObjectName(_fromUtf8("startButton"))
        self.stopButton = QtGui.QPushButton(self.centralWidget)
        self.stopButton.setGeometry(QtCore.QRect(260, 270, 113, 32))
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.label_2 = QtGui.QLabel(self.centralWidget)
        self.label_2.setGeometry(QtCore.QRect(430, 130, 59, 20))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.M2PWMSlider = QtGui.QSlider(self.centralWidget)
        self.M2PWMSlider.setGeometry(QtCore.QRect(60, 130, 361, 22))
        self.M2PWMSlider.setMaximum(127)
        self.M2PWMSlider.setOrientation(QtCore.Qt.Horizontal)
        self.M2PWMSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.M2PWMSlider.setObjectName(_fromUtf8("M2PWMSlider"))
        self.label_3 = QtGui.QLabel(self.centralWidget)
        self.label_3.setGeometry(QtCore.QRect(430, 180, 59, 20))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.M3PWMSlider = QtGui.QSlider(self.centralWidget)
        self.M3PWMSlider.setGeometry(QtCore.QRect(60, 180, 361, 22))
        self.M3PWMSlider.setMaximum(127)
        self.M3PWMSlider.setOrientation(QtCore.Qt.Horizontal)
        self.M3PWMSlider.setTickPosition(QtGui.QSlider.TicksBelow)
        self.M3PWMSlider.setObjectName(_fromUtf8("M3PWMSlider"))
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 510, 22))
        self.menuBar.setObjectName(_fromUtf8("menuBar"))
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtGui.QToolBar(MainWindow)
        self.mainToolBar.setObjectName(_fromUtf8("mainToolBar"))
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtGui.QStatusBar(MainWindow)
        self.statusBar.setObjectName(_fromUtf8("statusBar"))
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.label.setText(_translate("MainWindow", "M1", None))
        self.startButton.setText(_translate("MainWindow", "Start", None))
        self.stopButton.setText(_translate("MainWindow", "Stop", None))
        self.label_2.setText(_translate("MainWindow", "M2", None))
        self.label_3.setText(_translate("MainWindow", "M3", None))

