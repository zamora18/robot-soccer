import copy

from PyQt4 import QtGui, QtCore

import rospy, rostopic
from geometry_msgs.msg import Pose2D, Twist
from std_srvs.srv import Trigger
from playground.msg import BallState, RobotState, PIDInfo
from playground.srv import SetBool, SetBoolResponse, RoboClawRPC, RoboClawRPCResponse

import numpy as np

from matplotlib import pyplot as plt
from matplotlib import animation

import StepResponse

FIELD_LENGTH = 3.7
FIELD_WIDTH = 2.65
MAX_X_VEL = 2
MAX_Y_VEL = 2

class AllyUI(object):
    """docstring for AllyUI"""

    def __init__(self, ui, ally=None):
        super(AllyUI, self).__init__()

        if ally is None:
            return False

        if ally == 1:
            self.tbl_ball_pos = ui.tblBallPosition
            self.tbl_ball_future = ui.tblBallFuture
        else:
            self.tbl_ball_pos = None
            self.tbl_ball_future = None

        # Group thing
        self.groupbox = getattr(ui, 'groupAlly{}'.format(ally))
        self._my_title = self.groupbox.title()

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
        self.artists_field = {
            'position': None,
            'ball': None,
            'ball_predicted': None,
            'opponent': None
        }

        self.artists_vel = {
            'velocity': None
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

        # Create an artist for the current position (estimate) of the ally bot
        self.artists_field['position'] = canvas.ax.plot([],[], linewidth=0.6, animated=False)[0]

        # Create an artist for the desired position (unsafe) of the ally bot
        self.artists_field['desired'] = canvas.ax.plot([],[],'x',mfc='none',mec='g',mew=1.3,ms=7,animated=False)[0]

        # Create an artist for the desired position (safe) of the ally bot
        self.artists_field['desired_safe'] = canvas.ax.plot([],[],'2',mfc='none',mec='m',mew=1.3,ms=6,animated=False)[0]

        # Create an artist for the current position (estimate) of the ally bot
        self.artists_field['position_vision'] = canvas.ax.plot([],[],'d',mfc='none',mec='b',mew=0.75,ms=4,animated=False)[0]

        # Create an artist for the current position (estimate) of the ball
        self.artists_field['ball'] = canvas.ax.plot([],[],'o',mfc='none',mec='r',mew=1.2,ms=7,animated=False)[0]

        # Create an artist for the predicted position of the ball
        self.artists_field['ball_predicted'] = canvas.ax.plot([],[],'x',mfc='none',mec='g',mew=1,ms=5,animated=False)[0]

        # Create an artist for the current position (estimate) of the opponent
        self.artists_field['opponent'] = canvas.ax.plot([],[],'*',mfc='none',mec='c',mew=1,ms=7,animated=False)[0]

        # Create an artist for the current position (estimate) of the other opponent
        self.artists_field['other_opponent'] = canvas.ax.plot([],[],'*',mfc='none',mec='k',mew=0.8,ms=6,animated=False)[0]

        # Create an artist for the current position (estimate) of my ally
        self.artists_field['other_ally'] = canvas.ax.plot([],[],'d',mfc='none',mec='k',mew=0.8,ms=5,animated=False)[0]

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

        # Create an artist for the given velocity commands of ally
        self.artists_vel['velocity'] = canvas.ax.plot([],[], linewidth=0.75)[0]

    def _init_tables(self):
        self._init_generic_table(self.tbl_est_pos, \
                                header_labels=['xhat', 'yhat', 'thetahat'])

        self._init_generic_table(self.tbl_vel, \
                                header_labels=['vx', 'vy', 'vw'])

        self._init_generic_table(self.tbl_PID_error, \
                                header_labels=['x_e', 'y_e', 'theta_e'])

        self._init_generic_table(self.tbl_des_pos, editable=True, \
                                header_labels=['xhat_c', 'yhat_c', 'thetahat_c'])

        if self.tbl_ball_pos is not None:
            self._init_generic_table(self.tbl_ball_pos, \
                                    header_labels=['xhat', 'yhat'], cols=2, col_width=97)

        if self.tbl_ball_future is not None:
            self._init_generic_table(self.tbl_ball_future, \
                                    header_labels=['xhat_future', 'yhat_future'], cols=2, col_width=97)

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

    def update_table(self, tbl, cols, rounding=True):
        for idx, col in enumerate(cols):
            if rounding:
                cols[idx] = round(col,4)

            tbl.item(0,idx).setText(str(cols[idx]))

    def read_table(self, tbl):
        col1 = float(tbl.item(0,0).text())
        col2 = float(tbl.item(0,1).text())
        col3 = float(tbl.item(0,2).text())

        return (col1, col2, col3)

    def append_title(self, str=None):
        if str is not None:
            self.groupbox.setTitle('{} - {}'.format(self._my_title, str))
        else:
            self.groupbox.setTitle(self._my_title)


class Ally(object):
    """docstring for Ally"""
    def __init__(self, ui, ally=None, active=True, interval=250):
        super(Ally, self).__init__()

        if ally is None:
            return

        self.ally = ally

        # Create a dict so we now where we currently are
        self.current = {
            'velocity': None,
            'my_state': None,
            'opponent_state': None,
            'ball_state': None,
            'pidinfo': None,
            'desired_position': None,
            'desired_position_safe': None,
            'other_opponent_state': None,
            'other_ally_state': None
        }

        # Create a dictionary for last messages of different types
        self.last = copy.deepcopy(self.current)

        # Click to Drive positions (CTD)
        self.CTD_x1 = self.CTD_y1 = None
        self.CTD_x2 = self.CTD_y2 = None

        # Setup the UI for this ally
        self.ui = AllyUI(ui, ally=ally)

        # After we've set up the GUI, if this ally is not active just bail
        if not active:
            self.ui.append_title('Inactive')
            return

        # Placeholders for child windows
        self._step_resp = None

        # Figure out my namespace based on who I am
        ns = '/ally{}'.format(ally)
        self.ns = ns

        # If I am this ally, who is the other ally?
        other_ally = 1 if ally == 2 else 2
        
        # Connect ROS things
        rospy.Subscriber('{}/ally{}_state'.format(ns,ally), \
                            RobotState, self._handle_my_state)
        rospy.Subscriber('{}/ally{}_state'.format(ns,other_ally), \
                            RobotState, self._handle_other_ally_state)
        rospy.Subscriber('{}/opponent{}_state'.format(ns,ally), \
                            RobotState, self._handle_opponent_state)
        rospy.Subscriber('{}/opponent{}_state'.format(ns,other_ally), \
                            RobotState, self._handle_other_opponent_state)
        rospy.Subscriber('{}/ball_state'.format(ns), \
                            BallState, self._handle_ball_state)
        rospy.Subscriber('{}/desired_position'.format(ns), \
                            Pose2D, self._handle_des_pos)
        rospy.Subscriber('{}/desired_position_safe'.format(ns), \
                            Pose2D, self._handle_des_pos_safe)
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
        self.ui.btn_step_resp.clicked.connect(self._btn_step_resp)

        # Connect Plot Mouse events
        self.ui.plot_field.canvas.mpl_connect('button_press_event', self._plot_field_mouse_down)
        self.ui.plot_field.canvas.mpl_connect('motion_notify_event', self._plot_field_mouse_move)
        self.ui.plot_field.canvas.mpl_connect('button_release_event', self._plot_field_mouse_up)

        # matplotlib create animation
        # Look into using the same event_source for the two -- maybe this would help
        # decrease overhead/latency? See matplotlib source code for more info
        ## Interval - I had interval=8ms and everything was smooth, but CPU was
        ## at 130%. I changed it to 500ms and it went down to 30%! But the plotting
        ## was severly discretized. A good in between was at 250ms (~60% CPU), but if
        ## you're on a faster machine, you may want to increase interval to ~100ms
        self.animation_field = animation.FuncAnimation(self.ui.plot_field.canvas.fig, \
                            self._animate_field, init_func=self._animate_init_field, \
                            frames=None, interval=interval, blit=False, event_source=None)

        # self.animation_vel = animation.FuncAnimation(self.ui.plot_vel.canvas.fig, \
        #                     self._animate_vel, init_func=self._animate_init_vel, \
        #                     frames=None, interval=500, blit=False, event_source=None)

        # I couldn't figure out blitting. When set to True, I could get
        # CPU down to 20% from ~60%, but it left artifacts. It seems like
        # a pretty good direction to go down, but I'm unsure if it will work.

    # =========================================================================
    # ROS Event Callbacks (subscribers)
    # =========================================================================

    def _handle_my_state(self, msg):
        tbl = self.ui.tbl_est_pos
        self.ui.update_table(tbl, [msg.xhat, msg.yhat, msg.thetahat])

        # Save for later!
        self.last['my_state'] = self.current['my_state']
        self.current['my_state'] = msg

    def _handle_opponent_state(self, msg):
        # Save for later!
        self.last['opponent_state'] = self.current['opponent_state']
        self.current['opponent_state'] = msg

    def _handle_other_opponent_state(self, msg):
        # Save for later!
        self.last['other_opponent_state'] = self.current['other_opponent_state']
        self.current['other_opponent_state'] = msg

    def _handle_other_ally_state(self, msg):
        # Save for later!
        self.last['other_ally_state'] = self.current['other_ally_state']
        self.current['other_ally_state'] = msg

    def _handle_ball_state(self, msg):
        if self.ui.tbl_ball_pos is not None:
            tbl = self.ui.tbl_ball_pos
            self.ui.update_table(tbl, [msg.xhat, msg.yhat])

        if self.ui.tbl_ball_future is not None:
            tbl = self.ui.tbl_ball_future
            self.ui.update_table(tbl, [msg.xhat_future, msg.yhat_future])

        # Save for later!
        self.last['ball_state'] = self.current['ball_state']
        self.current['ball_state'] = msg

    def _handle_des_pos(self, msg):
        tbl = self.ui.tbl_des_pos
        self.ui.update_table(tbl, [msg.x, msg.y, msg.theta])

        # Save for later!
        self.last['desired_position'] = self.current['desired_position']
        self.current['desired_position'] = msg

    def _handle_des_pos_safe(self, msg):
        # Save for later!
        self.last['desired_position_safe'] = self.current['desired_position_safe']
        self.current['desired_position_safe'] = msg

    def _handle_vel(self, msg):
        tbl = self.ui.tbl_vel
        self.ui.update_table(tbl, [msg.linear.x, msg.linear.y, msg.angular.z])

        # Save for later!
        self.last['velocity'] = self.current['velocity']
        self.current['velocity'] = msg

    def _handle_PID_error(self, msg):
        tbl = self.ui.tbl_PID_error
        self.ui.update_table(tbl, [msg.error.x, msg.error.y, msg.error.theta])

        # Save for later!
        self.last['pidinfo'] = self.current['pidinfo']
        self.current['pidinfo'] = msg

        # if step_resp is active, add to the plot
        if self._step_resp is not None and self._step_resp.isVisible():
            self._step_resp.add_sample(msg)

    # =========================================================================
    # Qt Event Callbacks (buttons, plots, etc)
    # =========================================================================

    def _btn_clear(self):
        self._animate_init_field()
        # self._animate_init_vel() # There's no real need to clear this one

    def _btn_step_resp(self):
        if self._step_resp is None:
            self._step_resp = StepResponse.Dialog(self.ally)
        self._step_resp.show()

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
        msg = Pose2D()
        msg.x = self.current['my_state'].xhat
        msg.y = self.current['my_state'].yhat
        msg.theta = self.current['my_state'].thetahat
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
            theta_c = self.current['my_state'].thetahat

        # Send it off!
        msg = Pose2D()
        msg.x = self.CTD_x1
        msg.y = self.CTD_y1
        msg.theta = theta_c
        self.pub_des_pos.publish(msg)

        # Get ready for next time!
        self.CTD_x1 = None
        self.CTD_y1 = None

    # =========================================================================
    # matplotlib animation functions
    # =========================================================================

    def _update_line(self, artist, x, y, append=True):
        """Update Line
        """
        if append:
            xdata = artist.get_xdata()
            ydata = artist.get_ydata()

            xdata.append(x)
            ydata.append(y)
        else:
            xdata = [0, x]
            ydata = [0, y]

        artist.set_data(xdata, ydata)

    def _update_point(self, artist, x, y):
        """Update Point
        """
        artist.set_data(x, y)

    def _animate_init_field(self):
        for ax in self.ui.artists_field.itervalues():
            ax.set_data([],[])

        # Return the artists
        return self.ui.artists_field.itervalues()

    def _animate_field(self, i):
        # Check if roscore is still running on master machine
        try:
            rostopic.get_topic_class('/rosout')
        except rostopic.ROSTopicIOException as e:
            self.ui.append_title('Unable to connect to roscore...')
            return

        # ---------------------------------------------------------------------
        # -------------------------- Position ---------------------------------
        # ---------------------------------------------------------------------
        if self.current['my_state'] is not None:
            x = self.current['my_state'].xhat
            y = self.current['my_state'].yhat
            vision_x = self.current['my_state'].vision_x
            vision_y = self.current['my_state'].vision_y
            correction = self.current['my_state'].correction
        else:
            x, y = 0, 0
            vision_x, vision_y = 0, 0
            correction = False

        self._update_line(self.ui.artists_field['position'], x, y)

        if correction:
            self._update_point(self.ui.artists_field['position_vision'], vision_x, vision_y)

        # ---------------------------------------------------------------------
        # ----------------------- Desired Position ----------------------------
        # ---------------------------------------------------------------------
        if self.current['desired_position'] is not None:
            x = self.current['desired_position'].x
            y = self.current['desired_position'].y
        else:
            x, y = 0, 0

        self._update_point(self.ui.artists_field['desired'], x, y)

        # ---------------------------------------------------------------------
        # -------------------- Desired Position Safe --------------------------
        # ---------------------------------------------------------------------
        if self.current['desired_position_safe'] is not None:
            x = self.current['desired_position_safe'].x
            y = self.current['desired_position_safe'].y
        else:
            x, y = 0, 0

        self._update_point(self.ui.artists_field['desired_safe'], x, y)

        # ---------------------------------------------------------------------
        # -------------------------- Opponent ---------------------------------
        # ---------------------------------------------------------------------
        if self.current['opponent_state'] is not None:
            x = self.current['opponent_state'].xhat
            y = self.current['opponent_state'].yhat
        else:
            x, y = 0, 0

        self._update_point(self.ui.artists_field['opponent'], x, y)

        # ---------------------------------------------------------------------
        # ------------------ Ball Current / Predicted -------------------------
        # ---------------------------------------------------------------------
        if self.current['ball_state'] is not None:
            x = self.current['ball_state'].xhat
            y = self.current['ball_state'].yhat
            x_future = self.current['ball_state'].xhat_future
            y_future = self.current['ball_state'].yhat_future
        else:
            x, y = 0, 0
            x_future, y_future = 0, 0

        self._update_point(self.ui.artists_field['ball'], x, y)
        self._update_point(self.ui.artists_field['ball_predicted'], x_future, y_future)

        # ---------------------------------------------------------------------
        # ----------------------- Other Opponent ------------------------------
        # ---------------------------------------------------------------------
        if self.current['other_opponent_state'] is not None:
            x = self.current['other_opponent_state'].xhat
            y = self.current['other_opponent_state'].yhat
        else:
            x, y = 0, 0

        self._update_point(self.ui.artists_field['other_opponent'], x, y)

        # ---------------------------------------------------------------------
        # ------------------------- Other Ally --------------------------------
        # ---------------------------------------------------------------------
        if self.current['other_ally_state'] is not None:
            x = self.current['other_ally_state'].xhat
            y = self.current['other_ally_state'].yhat
        else:
            x, y = 0, 0

        self._update_point(self.ui.artists_field['other_ally'], x, y)

        ## When using `blit=True` and `animated=True`, this apparently needs
        ## to be uncommented. I feel like there is a better way however...
        # self.ui.plot_field.canvas.draw()

        # Return the artists!
        return self.ui.artists_field.itervalues()

    def _animate_init_vel(self):
        for ax in self.ui.artists_vel.itervalues():
            ax.set_data([],[])

        # Return the artists
        return self.ui.artists_vel.itervalues()

    def _animate_vel(self, i):
        # ---------------------------------------------------------------------
        # -------------------------- Velocity ---------------------------------
        # ---------------------------------------------------------------------
        if self.current['velocity'] is not None:
            x = self.current['velocity'].linear.x
            y = self.current['velocity'].linear.y
        else:
            x, y = 0, 0

        self._update_line(self.ui.artists_vel['velocity'], x, y, append=False)

        # Return the artists!
        return self.ui.artists_vel.itervalues()