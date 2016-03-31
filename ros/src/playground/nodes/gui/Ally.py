from PyQt4 import QtGui, QtCore

import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool
from playground.msg import BallState, RobotState
from playground.srv import SetBool, SetBoolResponse

import numpy as np

import matplotlib.pyplot as plt

FIELD_LENGTH = 4
FIELD_WIDTH = 3.2
MAX_X_VEL = 2
MAX_Y_VEL = 2

class AllyUI(object):
    """docstring for AllyUI"""

    # Plots
    plot_field = None
    plot_vel = None

    # Tables
    tbl_est_pos = None
    tbl_PID_error = None
    tbl_des_pos = None
    tbl_vel = None

    # Buttons
    btn_step_resp = None
    btn_clear = None
    btn_battery = None
    btn_kick = None
    btn_set_des_pos = None
    btn_stop_moving = None

    # Checkboxes
    chk_click_to_drive = None

    # Plots and axes
    axes = {
        'position': None,
        'velocity': None,
        'ball': None,
        'ball_predicted': None,
        'opponent': None
    }


    def __init__(self, ui, ally=None):
        super(AllyUI, self).__init__()

        if ally is None:
            return False

        # Plots
        self.plot_field = getattr(ui, 'plotAlly{}Field'.format(ally))
        self.plot_vel = getattr(ui, 'plotAlly{}Vel'.format(ally))

        # Tables
        self.tbl_est_pos = getattr(ui, 'tblAlly{}EstPos'.format(ally))
        self.tbl_PID_error = getattr(ui, 'tblAlly{}PIDError'.format(ally))
        self.tbl_des_pos = getattr(ui, 'tblAlly{}DesPos'.format(ally))
        self.tbl_vel = getattr(ui, 'tblAlly{}Vel'.format(ally))

        # Buttons
        self.btn_step_resp = getattr(ui, 'btnAlly{}StepResp'.format(ally))
        self.btn_clear = getattr(ui, 'btnAlly{}Clear'.format(ally))
        self.btn_battery = getattr(ui, 'btnAlly{}Battery'.format(ally))
        self.btn_kick = getattr(ui, 'btnAlly{}Kick'.format(ally))
        self.btn_set_des_pos = getattr(ui, 'btnAlly{}SetDesPos'.format(ally))
        self.btn_stop_moving = getattr(ui, 'btnAlly{}StopMoving'.format(ally))

        # Checkboxes
        self.chk_click_to_drive = getattr(ui, 'chkAlly{}CTD'.format(ally))

        # ---------------------------------------------------------------------
        # Initialize the GUI

        self._init_field()
        self._init_vel()
        self._init_tables()

    def _init_field(self):
        canvas = self.plot_field.canvas

        # Put the origin in the middle of the field
        xlim = (FIELD_LENGTH/2.0)
        ylim = (FIELD_WIDTH/2.0)
        canvas.ax.axis([-xlim, xlim, -ylim, ylim])

        # Turn on the grid to help see how the bot is driving
        canvas.ax.grid()

        # Clear all the labels set in `mpl_custom_widget`
        canvas.ax.set_xlabel('')
        canvas.ax.set_ylabel('')
        canvas.ax.set_title('')

        # Equal aspect ratio so it matches the world better
        canvas.ax.set_aspect('equal')

        # Try and get rid of as much of the margin/padding as possible
        canvas.fig.subplots_adjust(left=0.075, bottom=0.075, right=.95, top=.95)

        # Create an axis for the current position (estimate) of the ally bot
        self.axes['position'] = canvas.ax.plot([],[], linewidth=0.4)[0]

        # Create an axis for the current position (estimate) of the ball
        self.axes['ball'] = canvas.ax.plot([],[],'ro')[0]

        # Create an axis for the predicted position of the ball
        self.axes['ball_predicted'] = canvas.ax.plot([],[])[0]

        # Create an axis for the current position (estimate) of the opponent
        self.axes['opponent'] = canvas.ax.plot([],[])[0]

    def _init_vel(self):
        canvas = self.plot_vel.canvas

        # This will be a quiver (arrow) graph, so put limits
        # as maximum velocity in x,y directions, with origin in center
        xlim = MAX_X_VEL
        ylim = MAX_Y_VEL
        canvas.ax.axis([-xlim, xlim, -ylim, ylim])

        # Clear all the labels set in `mpl_custom_widget`
        canvas.ax.set_xlabel('')
        canvas.ax.set_ylabel('')
        canvas.ax.set_title('')

        # Equal aspect ratio so it matches the world better
        canvas.ax.set_aspect('equal')

        # Remove many of the tick marks so the plot is readable
        canvas.ax.set_xticks([-2, -1, 0, 1, 2])
        canvas.ax.set_yticks([-2, -1, 0, 1, 2])

        # Try and get rid of as much of the margin/padding as possible
        canvas.fig.subplots_adjust(left=0.125, bottom=0.10, right=.95, top=.95)

        # Create an axis for the given velocity commands of ally
        self.axes['velocity'] = canvas.ax.plot([0],[0], linewidth=0.4)[0]

    def _init_tables(self):
        self._init_generic_table(self.tbl_est_pos, \
                                header_labels=['xhat', 'yhat', 'thetahat'])

        self._init_generic_table(self.tbl_vel, \
                                header_labels=['vx', 'vy', 'vw'])

        self._init_generic_table(self.tbl_PID_error, \
                                header_labels=['x_e', 'y_e', 'theta_e'])

        self._init_generic_table(self.tbl_des_pos, editable=True, \
                                header_labels=['xhat_c', 'yhat_c', 'thetahat_c'])

    def _init_generic_table(self, tbl, header_labels=None, rows=1, cols=3, row_height=25, col_width=65, editable=False):
        tbl.setRowCount(rows)
        tbl.setRowHeight(0, row_height)
        tbl.setColumnCount(cols)

        for i in xrange(rows):
            tbl.setRowHeight(i, row_height)

        for i in xrange(cols):
            tbl.setColumnWidth(i, col_width)

        if header_labels is not None:
            tbl.setHorizontalHeaderLabels(header_labels)

        tbl.verticalHeader().hide()

        for i in xrange(rows):
            for j in xrange(cols):
                item = QtGui.QTableWidgetItem(str(0))
                if not editable:
                    item.setFlags(QtCore.Qt.ItemIsEnabled)
                tbl.setItem(i, j, item)

        # Change font size
        font = QtGui.QFont()
        font.setPointSize(9)
        tbl.setFont(font)
        tbl.horizontalHeader().setFont(font)

        # Don't allow column resizing
        tbl.horizontalHeader().setResizeMode(QtGui.QHeaderView.Fixed)

        # tbl.item(0,0).setText(str(5))

    def fast_redraw(self, canvas, plot):
        """Fast Redraw
        See: http://bastibe.de/2013-05-30-speeding-up-matplotlib.html
        """
        # For fast redrawing
        try:
            canvas.ax.draw_artist(plot)
            canvas.update()
            canvas.flush_events()
        except:
            pass

class Ally(object):
    """docstring for Ally"""
    def __init__(self, ui, ally=None):
        super(Ally, self).__init__()

        if ally is None:
            return False

        self.ally = ally

        # Setup the UI for this ally
        self.ui = AllyUI(ui, ally=ally)

        # Figure out my namespace based on who I am
        ns = '/ally{}'.format(ally)

        # self.ui.axes['position'].set_xdata(np.random.randn(20))
        
        # Connect ROS things
        rospy.Subscriber('{}/ally{}_state'.format(ns,ally), \
                            RobotState, self._handle_my_state)
        rospy.Subscriber('{}/ball_state'.format(ns,ally), \
                            BallState, self._handle_ball_state)

    def _handle_my_state(self, msg):
        plot = self.ui.axes['position']
        canvas = self.ui.plot_field.canvas

        xdata = plot.get_xdata()
        ydata = plot.get_ydata()

        xstart = [msg.xhat] if len(xdata) == 0 else [xdata[-1]]
        ystart = [msg.yhat] if len(ydata) == 0 else [ydata[-1]]

        xdata = np.concatenate( (xstart, [msg.xhat]) )
        ydata = np.concatenate( (ystart, [msg.yhat]) )

        plot.set_xdata(xdata)
        plot.set_ydata(ydata)

        self.ui.fast_redraw(canvas, plot)

        # from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
        # import ipdb; ipdb.set_trace()

    def _handle_opponent_state(self, msg):
        plot = self.ui.axes['position']
        canvas = self.ui.plot_field.canvas

        xdata = plot.get_xdata()
        ydata = plot.get_ydata()

        xstart = [msg.xhat] if len(xdata) == 0 else [xdata[-1]]
        ystart = [msg.yhat] if len(ydata) == 0 else [ydata[-1]]

        xdata = np.concatenate( (xstart, [msg.xhat]) )
        ydata = np.concatenate( (ystart, [msg.yhat]) )

        plot.set_data(xdata,ydata)
        # plot.set_ydata(ydata)

        self.ui.fast_redraw(canvas, plot)

        # from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
        # import ipdb; ipdb.set_trace()

    def _handle_ball_state(self, msg):
        plot = self.ui.axes['ball']
        canvas = self.ui.plot_field.canvas

        # if plot in canvas.ax.lines:
        #     del canvas.ax.lines[canvas.ax.lines.index(plot)]

        plot.set_xdata([msg.xhat])
        plot.set_ydata([msg.yhat])

        # circle = plt.Circle((msg.xhat,msg.yhat), 0.1, fc='r')
        # plot.axes.add_patch(circle)
        # circle.set_axes(canvas.ax)

        # canvas.ax.draw_artist(canvas.ax.patch)
        self.ui.fast_redraw(canvas, plot)
        canvas.draw_idle()

        # from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
        # import ipdb; ipdb.set_trace()