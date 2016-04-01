import sys
from PyQt4 import QtGui, uic, QtCore

import matplotlib

import roslib; #roslib.load_manifest('playground')
import rospy

import callbacks as cb

from Ally import Ally

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()

        # Load up the UI designed in QtCreator
        uic.loadUi('main.ui', self)

        # Setup ROS so ally's can use it
        rospy.init_node('command_center', anonymous=True)

        # Setup all the GUI and ROS elements for each Ally
        ally1 = Ally(self, ally=1)
        ally2 = Ally(self, ally=2)

        # Connect signals
        # self.btnAlly1Clear.clicked.connect(lambda x: cb.OK(ally1))



if __name__ == '__main__':
    # Set up Qt Application Window
    # QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_X11InitThreads)
    # matplotlib.use('TkAgg') # <-- THIS MAKES IT FAST!
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())

# from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
# import ipdb; ipdb.set_trace()