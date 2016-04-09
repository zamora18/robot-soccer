#!/usr/bin/env python
import os, sys
from PyQt4 import QtGui, uic, QtCore

from matplotlib import pyplot as plt
from matplotlib import animation

import numpy as np

import Ally

class Dialog(QtGui.QDialog):
    def __init__(self):
        super(Dialog, self).__init__()

        # Load up the UI designed in QtCreator
        absdir = os.path.dirname(os.path.abspath(__file__))
        uic.loadUi(os.path.join(absdir, 'step_response.ui'), self)

        # Load and initialize the UI
        self.ui = UI(self)

        # Connect to Qt buttons
        self.ui.btn_close.clicked.connect(self._btn_close)

        # Connect to matplotlib mouse events
        self.ui.plotX.canvas.mpl_connect('button_press_event', self._plot_mouse_press)
        self.ui.plotX.canvas.mpl_connect('button_release_event', self._plot_mouse_release)

        # matplotlib animation
        interval = 220
        self.animation_x = animation.FuncAnimation(self.ui.plotX.canvas.fig, \
                            self._animate_x, init_func=self._animate_init_x, \
                            frames=None, interval=interval, blit=False, event_source=None)

        self.samples = {
            'x': None,
            'y': None,
            'theta': None
        }

    # =========================================================================
    # Methods to add data samples
    # =========================================================================

    def add_sample(self, msg):
        self.samples['x'] = msg
        # (n, a) = self.ui.artists['xhat'].get_data()

        # # from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
        # # import ipdb; ipdb.set_trace()

        # n = np.concatenate( (n,[len(n)]) )
        # a = np.concatenate( (a,[msg.actual.x]) )

        # self.ui.artists['xhat'].set_data(n, a)

        # print (n, a)
        # self.ui.plotX.canvas.draw()
        # self.ui.artists['x'].canvas

    # =========================================================================
    # Qt Event Callbacks (buttons, plots, etc)
    # =========================================================================

    def _btn_close(self):
        self.done(0)

    def _plot_mouse_press(self,  event):
        print event

    def _plot_mouse_release(self,  event):
        print event

    def closeEvent(self, event):
        event.accept()

    # =========================================================================
    # Matplotlib animation stuff
    # =========================================================================

    def _update_line(self, artist, x=None, y=None, append=True):
        """Update Line
        """
        if append:
            xdata = artist.get_xdata()
            ydata = artist.get_ydata()

            if x is not None:
                xdata.append(x)
            else:
                xdata.append(len(xdata))

            if y is not None:
                ydata.append(y)
            else:
                ydata.append(len(ydata))
        else:
            xdata = [0, x]
            ydata = [0, y]

        artist.set_data(xdata, ydata)

        return len(xdata)

    def _animate_init_x(self):
        # for ax in self.ui.artists_field.itervalues():
        self.ui.artists['x'].set_data([],[])
        self.ui.artists['x_c'].set_data([],[])

        # Return the artists
        return [self.ui.artists['x'], self.ui.artists['x_c']]

    def _animate_x(self, i):

        if self.samples['x'] is not None:
            x_c = self.samples['x'].desired.x
            x = self.samples['x'].actual.x
        else:
            x = 0

        n = self._update_line(self.ui.artists['x_c'], y=x_c)
        n = self._update_line(self.ui.artists['x'], y=x)

        self.ui.plotX.canvas.ax.set_xlim(0, 2*n)

        # Return the artists
        return [self.ui.artists['x'], self.ui.artists['x_c']]



class UI(object):
    """docstring for UI"""
    def __init__(self, ui):
        super(UI, self).__init__()

        # Entire dialog window handle
        self.window = ui

        # Plots
        self.plotX = ui.plotStepRespX
        self.plotY = ui.plotStepRespY
        self.plotTheta = ui.plotStepRespTheta

        # Buttons
        self.btn_save = ui.btnSave
        self.btn_close = ui.btnClose

        self.artists = {
            'x': None,
            'x_c': None,
            'y': None,
            'y_c': None,
            'theta': None,
            'theta_c': None
        }

        # ---------------------------------------------------------------------
        # Initialize the GUI

        self._init_step_response()
        
    
    def _init_step_response(self):
        canvasX = self.plotX.canvas
        canvasY = self.plotY.canvas
        canvasTheta = self.plotTheta.canvas

        # I should remove these toolbars and then use mouse click/drag to zoom
        self.plotX.toolbar.show()
        self.plotY.toolbar.show()
        self.plotTheta.toolbar.show()

        for canvas, pos in [(canvasX, 'x'), (canvasY, 'y'), (canvasTheta, 'theta')]:
            # Put the origin in the middle of the field
            xlim = (Ally.FIELD_LENGTH/2.0 + 1)
            ylim = (Ally.FIELD_WIDTH/2.0 + 1)
            canvas.ax.axis([-xlim, xlim, -ylim, ylim])

            # Turn on the grid to help see how the bot is driving
            canvas.ax.grid()

            # Clear all the labels set in `mpl_custom_widget`
            canvas.ax.set_xlabel('samples (n)')
            canvas.ax.set_ylabel('{}-position (m)'.format(pos))
            canvas.ax.set_title('')

            # Try and get rid of as much of the margin/padding as possible
            canvas.fig.subplots_adjust(left=0.075, bottom=0.075, right=.95, top=.95)

            # Create an artist for this position's PID info
            self.artists['{}_c'.format(pos)] = canvas.ax.plot([],[], 'b', linewidth=0.6, animated=False)[0]
            self.artists['{}'.format(pos)] = canvas.ax.plot([],[], 'r', linewidth=0.6, animated=False)[0]