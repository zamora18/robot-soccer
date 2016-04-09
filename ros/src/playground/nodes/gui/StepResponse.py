#!/usr/bin/env python
import os, sys
from PyQt4 import QtGui, uic, QtCore

from matplotlib import pyplot as plt
from matplotlib import animation

import numpy as np

import Ally

class Dialog(QtGui.QDialog):
    def __init__(self, ally):
        super(Dialog, self).__init__()

        # Load up the UI designed in QtCreator
        absdir = os.path.dirname(os.path.abspath(__file__))
        uic.loadUi(os.path.join(absdir, 'step_response.ui'), self)

        # Load and initialize the UI
        self.ui = UI(self, ally)

        # Connect to Qt buttons
        self.ui.btn_close.clicked.connect(self._btn_close)
        self.ui.btn_clear.clicked.connect(self._btn_clear)
        self.ui.btn_toggle_pause.clicked.connect(self._btn_toggle_pause)

        # Connect to matplotlib mouse events
        self.ui.plot.canvas.mpl_connect('button_press_event', self._plot_mouse_press)
        self.ui.plot.canvas.mpl_connect('button_release_event', self._plot_mouse_release)

        # matplotlib animation
        interval = 220
        self.animation_timer = self.ui.plot.canvas.fig.canvas.new_timer()
        self.animation_timer.interval = interval
        self.animation = animation.FuncAnimation(self.ui.plot.canvas.fig, \
                            self._animate, init_func=self._animate_init, \
                            frames=None, interval=interval, blit=False, event_source=self.animation_timer)

        self.last_sample = None

    # =========================================================================
    # Methods to add data samples
    # =========================================================================

    def add_sample(self, msg):
        self.last_sample = msg

    # =========================================================================
    # Qt Event Callbacks (buttons, plots, etc)
    # =========================================================================

    def _btn_close(self):
        self.close()

    def _btn_clear(self):
        self._animate_init()

    def _btn_toggle_pause(self):
        if self.ui.btn_toggle_pause.text() == 'Pause':
            self.animation_timer.stop()
            self.ui.btn_toggle_pause.setText('Play')
        else:
            self.animation_timer.start()
            self.ui.btn_toggle_pause.setText('Pause')

    def _plot_mouse_press(self,  event):
        print event

    def _plot_mouse_release(self,  event):
        print event

    def closeEvent(self, event):
        self.animation_timer.stop()
        self._animate_init()
        event.accept()

    def show(self):
        self.animation_timer.start()
        super(Dialog, self).show()

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

    def _animate_init(self):
        for artist in self.ui.artists.itervalues():
            artist.set_data([],[])

        # Return the artists
        return self.ui.artists.itervalues()

    def _animate(self, i):

        if self.last_sample is not None:
            x_c = self.last_sample.desired.x
            x = self.last_sample.actual.x

            y_c = self.last_sample.desired.y
            y = self.last_sample.actual.y

            theta_c = self.last_sample.desired.theta
            theta = self.last_sample.actual.theta
        else:
            x_c = x = 0
            y_c = y = 0
            theta_c = theta = 0

        n = self._update_line(self.ui.artists['x_c'], y=x_c)
        n = self._update_line(self.ui.artists['x'], y=x)
        self.ui.plot.canvas.ax['x'].set_xlim(0, 2*n)

        n = self._update_line(self.ui.artists['y_c'], y=y_c)
        n = self._update_line(self.ui.artists['y'], y=y)
        self.ui.plot.canvas.ax['y'].set_xlim(0, 2*n)

        n = self._update_line(self.ui.artists['theta_c'], y=theta_c)
        n = self._update_line(self.ui.artists['theta'], y=theta)
        self.ui.plot.canvas.ax['theta'].set_xlim(0, 2*n)

        # Return the artists
        return self.ui.artists.itervalues()



class UI(object):
    """docstring for UI"""
    def __init__(self, ui, ally):
        super(UI, self).__init__()

        # Entire dialog window handle
        self.window = ui

        self.window.setWindowTitle('Step Response: Ally {}'.format(ally))

        # Plots
        self.plot = ui.plotStepResp

        # Buttons
        self.btn_save = ui.btnSave
        self.btn_close = ui.btnClose
        self.btn_toggle_pause = ui.btnToggle
        self.btn_clear = ui.btnClear

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
        canvas = self.plot.canvas

        # I should remove these toolbars and then use mouse click/drag to zoom
        self.plot.toolbar.show()

        canvas.ax = {
            'x': canvas.fig.add_subplot(311),
            'y': canvas.fig.add_subplot(312),
            'theta': canvas.fig.add_subplot(313),
        }

        xlim = (Ally.FIELD_LENGTH/2.0 + 1)
        ylim = (Ally.FIELD_WIDTH/2.0 + 1)

        # Try and get rid of as much of the margin/padding as possible
        canvas.fig.subplots_adjust(left=0.075, bottom=0.075, right=.95, top=.95)

        canvas.format_labels()

        for pos, ax in canvas.ax.iteritems():
            # Put the origin in the middle of the plot
            if pos == 'theta':
                ax.axis([-xlim, xlim, -360, 360])
            else:
                ax.axis([-xlim, xlim, -ylim, ylim])

            # Turn on the grid to help see how the bot is driving
            ax.grid()

            # Clear all the labels set in `mpl_custom_widget`
            ax.set_xlabel('samples (n)')
            ax.set_ylabel('{}-position (m)'.format(pos))
            ax.set_title('')

            # Create an artist for this position's PID info
            self.artists['{}_c'.format(pos)] = ax.plot([],[], 'b', linewidth=0.6, animated=False)[0]
            self.artists['{}'.format(pos)] = ax.plot([],[], 'r', linewidth=0.6, animated=False)[0]