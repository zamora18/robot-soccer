import os, sys
from PyQt4 import QtGui, uic, QtCore

import matplotlib

import roslib; roslib.load_manifest('playground')
import rospy

from Ally import Ally

class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        absdir = os.path.dirname(os.path.abspath(__file__))

        # Load up the UI designed in QtCreator
        uic.loadUi(os.path.join(absdir, 'window.ui'), self)

        # Setup ROS so ally's can use it
        rospy.init_node('command_center', anonymous=True)

        # Setup all the GUI and ROS elements for each Ally
        ally1 = Ally(self, ally=1, active=True)
        ally2 = Ally(self, ally=2, active=True)

if __name__ == '__main__':
    # Set up Qt Application Window
    # QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_X11InitThreads)
    app = QtGui.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

# from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
# import ipdb; ipdb.set_trace()