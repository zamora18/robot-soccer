#!/usr/bin/env python
import os, sys
from PyQt4 import QtGui, uic, QtCore

import matplotlib

class Dialog(QtGui.QDialog):
    def __init__(self):
        super(Dialog, self).__init__()

        absdir = os.path.dirname(os.path.abspath(__file__))

        # Load up the UI designed in QtCreator
        uic.loadUi(os.path.join(absdir, 'step_response.ui'), self)

        self.ui = UI(self)


class UI(object):
    """docstring for UI"""
    def __init__(self, ui):
        super(UI, self).__init__()

        # Entire dialog window handle
        self.window = ui

        # Plots
        self.plot = ui.plotStepResp

        # Buttons
        self.btn_save = ui.btnSave
        self.btn_close = ui.btnClose

        # ---------------------------------------------------------------------
        # Initialize the GUI

        self._init_step_response()
        
    
    def _init_step_response(self):
        canvas = self.plot.canvas

        canvas.fig.delaxes(canvas.ax)

        canvas.ax = {
            'x': canvas.fig.add_subplot(311),
            'y': canvas.fig.add_subplot(312),
            'theta': canvas.fig.add_subplot(313),
        }

        # Put the origin in the middle of the field
        # xlim = (FIELD_LENGTH/2.0)
        # ylim = (FIELD_WIDTH/2.0)
        # canvas.ax.axis([-xlim, xlim, -ylim, ylim])

        # Turn on the grid to help see how the bot is driving
        canvas.ax['x'].grid()

        # Clear all the labels set in `mpl_custom_widget`
        canvas.ax['x'].set_xlabel('samples (n)')
        canvas.ax['x'].set_ylabel('x-position (m)')
        canvas.ax['x'].set_title('')

        # Equal aspect ratio so it matches the world better
        canvas.ax['x'].set_aspect('equal')

        # Try and get rid of as much of the margin/padding as possible
        canvas.fig.subplots_adjust(left=0.075, bottom=0.075, right=.95, top=.95)