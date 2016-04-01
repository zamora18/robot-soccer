from PyQt4 import QtGui, QtCore

import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_srvs.srv import Trigger
from playground.msg import BallState, RobotState, PIDInfo
from playground.srv import SetBool, SetBoolResponse, RoboClawRPC, RoboClawRPCResponse

import numpy as np

import matplotlib.pyplot as plt

FIELD_LENGTH = 4
FIELD_WIDTH = 3.2
MAX_X_VEL = 2
MAX_Y_VEL = 2

class AllyUI(object):
    """docstring for AllyUI"""

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

        # Plots and axes
        self.axes = {
            'position': None,
            'velocity': None,
            'ball': None,
            'ball_predicted': None,
            'opponent': None
        }

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
        self.axes['ball'] = None #canvas.ax.plot([],[],'ro')[0]

        # Create an axis for the predicted position of the ball
        self.axes['ball_predicted'] = None #canvas.ax.plot([],[])[0]

        # Create an axis for the current position (estimate) of the opponent
        self.axes['opponent'] = None #canvas.ax.plot([],[])[0]

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
        self.axes['velocity'] = None #canvas.ax.plot([0],[0], linewidth=0.4)[0]

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

    def update_table(self, tbl, col1, col2, col3, rounding=True):
        if rounding:
            col1 = round(col1,4)
            col2 = round(col2,4)
            col3 = round(col3,4)

        tbl.item(0,0).setText(str(col1))
        tbl.item(0,1).setText(str(col2))
        tbl.item(0,2).setText(str(col3))

    def read_table(self, tbl):
        col1 = float(tbl.item(0,0).text())
        col2 = float(tbl.item(0,1).text())
        col3 = float(tbl.item(0,2).text())

        return (col1, col2, col3)

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

    def draw_circle(self, canvas, circle, new_center, color='r', size=0.05):
        """Draw Circle
        """
        tolerance = 0.001 # 1mm

        # Enough has changed that we should plot
        plot = True

        # Remove last circle
        if circle is not None:           
            # is the new center much different than the old one?
            if abs(new_center[0]-circle.center[0]) < tolerance and \
                    abs(new_center[1]-circle.center[1]) < tolerance:
                plot = False

            if plot:
                # Fill in old circle
                old = plt.Circle(circle.center, (size+0.015), fc='w',ec='w',fill=True)
                canvas.ax.add_artist(old)
                self.fast_redraw(canvas,old)
                canvas.ax.artists.remove(old)

                # circle.remove()
                canvas.ax.artists.remove(circle)

                # canvas.ax.draw_idle()

        if plot:
            # Create new circle
            circle = plt.Circle(new_center, size, fc=color, ec=color, fill=False)

            canvas.ax.add_artist(circle)
            self.fast_redraw(canvas, circle)

            return circle

        # plot.set_xdata([msg.xhat])
        # plot.set_ydata([msg.yhat])

        # circle = plt.Circle((msg.xhat,msg.yhat), 0.1, fc='r')
        # plot.axes.add_patch(circle)
        # circle.set_axes(canvas.ax)

        # canvas.ax.draw_artist(canvas.ax.patch)
        
        # canvas.ax.add_artist(self.ui.axes['position'])
        # canvas.ax.redraw_in_frame()
        # ball.axes.draw_idle()
        # canvas.ax.draw_artist(ball)
        # canvas.update()
        # canvas.flush_events()
        

        # from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
        # import ipdb; ipdb.set_trace()


    def draw_diamond(self, canvas, rect, new_corner, color='r', size=0.05):
        """Draw Diamond
        """
        tolerance = 0.001 # 1mm

        # Enough has changed that we should plot
        plot = True

        # Remove last rect
        if rect is not None:           
            # is the new corner much different than the old one?
            if abs(new_corner[0]-rect.xy[0]) < tolerance and \
                    abs(new_corner[1]-rect.xy[1]) < tolerance:
                plot = False

            if plot:
                # Fill in old rect
                old = plt.Rectangle(rect.xy,(size+0.02),(size+0.02),angle=45.0,fc='w',ec='w',fill=True)
                canvas.ax.add_artist(old)
                self.fast_redraw(canvas,old)
                canvas.ax.artists.remove(old)

                # rect.remove()
                canvas.ax.artists.remove(rect)

                # canvas.ax.draw_idle()

        if plot:
            # Create new rect
            rect = plt.Rectangle(new_corner, size, size,angle=45.0,fc=color,ec=color,fill=False)

            canvas.ax.add_artist(rect)
            self.fast_redraw(canvas, rect)

            return rect

    def draw_arrow_from_origin(self, canvas, arrow, old_endpoint, endpoint, color='b', width=0.05):
        """Draw Arrow from Origin

            This doesn't do a great job of removing old velocity vectors.
            But I'm too lazy right now.

            A better solution may be to draw a square over most of the area
            of the plot. Or figure out how to make `canvas.ax.redraw_in_frame()`
            play nicely.
        """
        tolerance = 0.001 # 1mm

        # Enough has changed that we should plot
        plot = True

        # Remove last arrow
        if arrow is not None:           
            # is the new corner much different than the old one?
            if abs(endpoint[0]-old_endpoint[0]) < tolerance and \
                    abs(endpoint[1]-old_endpoint[1]) < tolerance:
                plot = False

            if plot:
                # Fill in old arrow
                dx = old_endpoint[0] + np.sign(old_endpoint[0])*0.5
                dy = old_endpoint[1] + np.sign(old_endpoint[1])*0.5
                old = plt.Arrow(0,0,dx,dy,width=(width*5),fc='w',ec='w',fill=True)
                canvas.ax.add_artist(old)
                self.fast_redraw(canvas,old)
                canvas.ax.artists.remove(old)

                # arrow.remove()
                canvas.ax.artists.remove(arrow)

                # canvas.ax.draw_idle()

        if plot:
            # Create new arrow
            arrow = plt.Arrow(0, 0, *endpoint, width=width,fc=color,ec=color,fill=False)

            canvas.ax.add_artist(arrow)
            self.fast_redraw(canvas, arrow)

            return arrow


class Ally(object):
    """docstring for Ally"""
    def __init__(self, ui, ally=None):
        super(Ally, self).__init__()

        if ally is None:
            return False

        self.ally = ally

        # Create a dictionary for last messages of different types
        self.last = {
            'velocity': None,
            'position': None
        }

        # Click to Drive positions (CTD)
        self.CTD_x1 = self.CTD_y1 = None
        self.CTD_x2 = self.CTD_y2 = None

        # Setup the UI for this ally
        self.ui = AllyUI(ui, ally=ally)

        # Figure out my namespace based on who I am
        ns = '/ally{}'.format(ally)
        self.ns = ns
        
        # Connect ROS things
        rospy.Subscriber('{}/ally{}_state'.format(ns,ally), \
                            RobotState, self._handle_my_state)
        rospy.Subscriber('{}/opponent{}_state'.format(ns,ally), \
                            RobotState, self._handle_opponent_state)
        rospy.Subscriber('{}/ball_state'.format(ns), \
                            BallState, self._handle_ball_state)
        rospy.Subscriber('{}/desired_position'.format(ns), \
                            Pose2D, self._handle_des_pos)
        rospy.Subscriber('{}/vel_cmds'.format(ns), \
                            Twist, self._handle_vel)
        rospy.Subscriber('{}/pidinfo'.format(ns), \
                            PIDInfo, self._handle_PID_error)

        self.pub_des_pos = rospy.Publisher('{}/desired_position'.format(ns), \
                            Pose2D, queue_size=10)

        # Connect Qt Buttons
        self.ui.btn_clear.clicked.connect(self._btn_clear)
        self.ui.btn_kick.clicked.connect(self._btn_kick)
        self.ui.btn_battery.clicked.connect(self._btn_battery)
        self.ui.btn_set_des_pos.clicked.connect(self._btn_des_pos)
        self.ui.btn_stop_moving.clicked.connect(self._btn_stop_moving)

        # Connect Plot Mouse events
        self.ui.plot_field.canvas.mpl_connect('button_press_event', self._plot_field_mouse_down)
        self.ui.plot_field.canvas.mpl_connect('motion_notify_event', self._plot_field_mouse_move)
        self.ui.plot_field.canvas.mpl_connect('button_release_event', self._plot_field_mouse_up)

    # =========================================================================
    # ROS Event Callbacks (subscribers)
    # =========================================================================

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

        tbl = self.ui.tbl_est_pos
        self.ui.update_table(tbl, msg.xhat, msg.yhat, msg.thetahat)

        self.last['position'] = msg

    def _handle_opponent_state(self, msg):
        opponent = self.ui.axes['opponent']
        canvas = self.ui.plot_field.canvas

        artist = self.ui.draw_circle(canvas, opponent, \
                        (msg.xhat,msg.yhat), color='c', size=0.05)
        if artist is not None:
            self.ui.axes['opponent'] = artist


    def _handle_ball_state(self, msg):
        ball = self.ui.axes['ball']
        ball_predicted = self.ui.axes['ball_predicted']
        canvas = self.ui.plot_field.canvas

        artist = self.ui.draw_circle(canvas, ball, \
                        (msg.xhat,msg.yhat), color='r', size=0.05)
        if artist is not None:
            self.ui.axes['ball'] = artist

        artist = self.ui.draw_circle(canvas, ball_predicted, \
                        (msg.xhat_future,msg.yhat_future), color='g', size=0.02)
        if artist is not None:
            self.ui.axes['ball_predicted'] = artist

    def _handle_des_pos(self, msg):
        tbl = self.ui.tbl_des_pos
        self.ui.update_table(tbl, msg.x, msg.y, msg.theta)

    def _handle_vel(self, msg):
        velocity = self.ui.axes['velocity']
        old = self.last['velocity']
        canvas = self.ui.plot_vel.canvas

        last_x = old.linear.x if old is not None else 0
        last_y = old.linear.y if old is not None else 0

        artist = self.ui.draw_arrow_from_origin(canvas, velocity, \
                        (last_x, last_y), \
                        (msg.linear.x,msg.linear.y), color='b', width=0.2)
        if artist is not None:
            self.ui.axes['velocity'] = artist

        self.last['velocity'] = msg

        tbl = self.ui.tbl_vel
        self.ui.update_table(tbl, msg.linear.x, msg.linear.y, msg.angular.z)

    def _handle_PID_error(self, msg):
        tbl = self.ui.tbl_PID_error
        self.ui.update_table(tbl, msg.error.x, msg.error.y, msg.error.theta)


    # =========================================================================
    # Qt Event Callbacks (buttons, plots, etc)
    # =========================================================================

    def _btn_clear(self):
        self.ui.plot_field.canvas.draw()
        # self.ui.plot_vel.canvas.draw() # this makes it too slow...

    def _btn_kick(self):
        try:
            kick_srv = rospy.ServiceProxy('{}/kick'.format(self.ns), Trigger)
            kick_srv()
        except rospy.ServiceException, e:
            print "Kick service call failed: %s"%e

    def _btn_battery(self):
        try:
            battery_srv = rospy.ServiceProxy('{}/battery'.format(self.ns), RoboClawRPC)
            v = battery_srv().message
            self.ui.btn_battery.setText('Battery: {}v'.format(v))
        except rospy.ServiceException, e:
            print "Battery service call failed: %s"%e

    def _btn_des_pos(self):
        tbl = self.ui.tbl_des_pos
        (x_c, y_c, theta_c) = self.ui.read_table(tbl)

        msg = Pose2D()
        msg.x = x_c
        msg.y = y_c
        msg.theta = theta_c
        self.pub_des_pos.publish(msg)

    def _btn_stop_moving(self):
        tbl = self.ui.tbl_est_pos
        (xhat, yhat, thetahat) = self.ui.read_table(tbl)

        msg = Pose2D()
        msg.x = xhat
        msg.y = yhat
        msg.theta = thetahat
        self.pub_des_pos.publish(msg)

    def _plot_field_mouse_down(self, event):
        if self.ui.chk_click_to_drive.isChecked():

            # Get the commanded click data for later
            self.CTD_x1 = event.xdata
            self.CTD_y1 = event.ydata

    def _plot_field_mouse_move(self, event):
        # No business being here if you didn't come through mouse_down
        if self.CTD_x1 is None or self.CTD_y1 is None:
            return

        self.CTD_x2 = event.xdata
        self.CTD_y2 = event.ydata

    def _plot_field_mouse_up(self, event):
        # No business being here if you didn't come through mouse_down
        if self.CTD_x1 is None or self.CTD_y1 is None:
            return

        if self.CTD_x2 is not None and self.CTD_y2 is not None:
            theta_c = np.arctan2(self.CTD_y2-self.CTD_y1, self.CTD_x2-self.CTD_x1)*180/np.pi

            # Get ready for next time
            self.CTD_x2 = None
            self.CTD_y2 = None
        else:
            theta_c = self.last['position'].thetahat

        # Send it off!
        msg = Pose2D()
        msg.x = self.CTD_x1
        msg.y = self.CTD_y1
        msg.theta = theta_c
        self.pub_des_pos.publish(msg)

        # Get ready for next time!
        self.CTD_x1 = None
        self.CTD_y1 = None