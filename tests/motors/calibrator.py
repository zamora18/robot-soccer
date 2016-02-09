import sys
from PyQt4 import QtCore, QtGui
from calibrator_window import Ui_MainWindow

import CalibratorSM

import matplotlib.pyplot as plt
import numpy as np

class CalibratorDialog(QtGui.QDialog):
    

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Connect UI events to code
        self.ui.startButton.clicked.connect(self.start_calibration)
        self.ui.stopButton.clicked.connect(self.stop_calibration)
        self.ui.M1PWMSlider.valueChanged.connect(self.pwm_changed_m1)
        self.ui.M2PWMSlider.valueChanged.connect(self.pwm_changed_m2)
        self.ui.M3PWMSlider.valueChanged.connect(self.pwm_changed_m3)

        # init calibrator
        CalibratorSM.init()

        # Set sliders to default values
        self.ui.M1PWMSlider.setValue(CalibratorSM.DEFAULT_SPEED_M1)
        self.ui.M2PWMSlider.setValue(CalibratorSM.DEFAULT_SPEED_M2)
        self.ui.M3PWMSlider.setValue(CalibratorSM.DEFAULT_SPEED_M3)

        # Timer stuff
        self.tick_timer = QtCore.QTimer()
        self.tick_timer.timeout.connect(CalibratorSM.tick)
        self.tick_timer.start(CalibratorSM.timer_rate_ms)

        self.ui_timer = QtCore.QTimer()
        self.ui_timer.timeout.connect(self._update_ui)
        self.ui_timer.start(250)

    def _update_ui(self):
        pass

    def start_calibration(self):
        CalibratorSM.start()

    def stop_calibration(self):
        CalibratorSM.stop()

    def pwm_changed_m1(self):
        val = self.ui.M1PWMSlider.value()
        CalibratorSM.set_m1_speed(val)

    def pwm_changed_m2(self):
        val = self.ui.M2PWMSlider.value()
        CalibratorSM.set_m2_speed(val)

    def pwm_changed_m3(self):
        val = self.ui.M3PWMSlider.value()
        CalibratorSM.set_m3_speed(val)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)

    # plt.axis([0, 100, 0, 400000])
    # plt.ion()
    # plt.show()
    # timer.start(500)
    myapp = CalibratorDialog()
    myapp.show()
    sys.exit(app.exec_())
