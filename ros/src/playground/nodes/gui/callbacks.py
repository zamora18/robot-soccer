import numpy as np
import matplotlib.pyplot as plt
import pyqtgraph as pg

def OK(ally1):
    canvas = ally1.ui.plot_field.canvas

    canvas.draw()

    # from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
    # import ipdb; ipdb.set_trace()