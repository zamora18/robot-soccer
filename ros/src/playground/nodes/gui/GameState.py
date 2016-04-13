#!/usr/bin/env python
import os, sys
from PyQt4 import QtGui, uic, QtCore

import rospy, rostopic
from playground.msg import GameState as GameStateMsg

from matplotlib import pyplot as plt
from matplotlib import animation

import numpy as np

class UI(object):
    """docstring for UI"""
    def __init__(self, ui):
        super(UI, self).__init__()

        # Buttons
        self.btn_toggle_spacebar = ui.btnSpacebarToggle
        self.btn_us_goal = ui.btnUsGoal
        self.btn_them_goal = ui.btnThemGoal

        # Radio Buttons
        self.radio_one_v_one = ui.radOne
        self.radio_two_v_two = ui.radTwo

        # Labels
        self.lbl_us_score = ui.lblUsScore
        self.lbl_them_score = ui.lblThemScore

    def change_toggle_text(self, play=None):
        # What is the current button text?
        txt = self.btn_toggle_spacebar.text()

        # What are the'Spacebar Toggle (Pause and Go Home)' text that should be?
        PLAY = 'Spacebar Toggle (Pause and Go Home)'
        PAUSED = 'Spacebar Toggle (Turn on AI and play!)'

        if play is not None:
            txt = PLAY if play else PAUSED
            self.btn_toggle_spacebar.setText(txt)
            return

        if txt == PAUSED:
            self.btn_toggle_spacebar.setText(PLAY)
        else:
            self.btn_toggle_spacebar.setText(PAUSED)

class GameState(object):
    def __init__(self, ui):
        super(GameState, self).__init__()

        # Load and initialize the UI
        self.ui = UI(ui)

        # Connect ROS things
        rospy.Subscriber('/game_state', GameStateMsg, self._handle_game_state)
        self.pub_game_state = rospy.Publisher('/game_state', GameStateMsg, queue_size=10)

        # Connect to Qt buttons
        self.ui.btn_toggle_spacebar.clicked.connect(self._btn_toggle_spacebar)
        self.ui.btn_us_goal.clicked.connect(self._btn_us_goal)
        self.ui.btn_them_goal.clicked.connect(self._btn_them_goal)

        # Connect to Qt Radio Buttons
        self.ui.radio_one_v_one.toggled.connect(self._radio_one_v_one)
        self.ui.radio_two_v_two.toggled.connect(self._radio_two_v_two)

        # Initialize Game State
        self.game_state = {
            'play': False,
            'two_v_two': False,
            'us_score': 0,
            'them_score': 0
        }

    # =========================================================================
    # ROS Event Callbacks (subscribers)
    # =========================================================================

    def _handle_game_state(self, msg):
        self.game_state['play'] = msg.play
        self.game_state['two_v_two'] = msg.two_v_two

        self.ui.change_toggle_text(play=msg.play)

        if not msg.two_v_two:
            self.ui.radio_one_v_one.setChecked(True)
        else:
            self.ui.radio_two_v_two.setChecked(True)

        if msg.usgoal:
            self._increment_our_score()

        if msg.themgoal:
            self._increment_their_score()

    # =========================================================================
    # Qt Event Callbacks (buttons, plots, etc)
    # =========================================================================

    def _btn_toggle_spacebar(self):
        # Toggle play
        self.game_state['play'] = not self.game_state['play']

        self.ui.change_toggle_text()

        msg = GameStateMsg()
        msg.play = self.game_state['play']
        msg.two_v_two = self.game_state['two_v_two']
        msg.usgoal = False
        msg.themgoal = False
        self.pub_game_state.publish(msg)

    def _btn_us_goal(self):
        msg = GameStateMsg()
        msg.play = self.game_state['play']
        msg.two_v_two = self.game_state['two_v_two']
        msg.usgoal = True
        msg.themgoal = False
        self.pub_game_state.publish(msg)

    def _btn_them_goal(self):
        msg = GameStateMsg()
        msg.play = self.game_state['play']
        msg.two_v_two = self.game_state['two_v_two']
        msg.usgoal = False
        msg.themgoal = True
        self.pub_game_state.publish(msg)

    def _radio_one_v_one(self):
        self.game_state['two_v_two'] = not self.ui.radio_one_v_one.isChecked()

    def _radio_two_v_two(self):
        self.game_state['two_v_two'] = self.ui.radio_two_v_two.isChecked()

    # =========================================================================
    # Other
    # =========================================================================

    def _increment_our_score(self):
        self.game_state['us_score'] += 1
        self.ui.lbl_us_score.setText(str(self.game_state['us_score']))

    def _increment_their_score(self):
        self.game_state['them_score'] += 1
        self.ui.lbl_them_score.setText(str(self.game_state['them_score']))