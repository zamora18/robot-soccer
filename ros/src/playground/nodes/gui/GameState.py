#!/usr/bin/env python
import os, sys
from PyQt4 import QtGui, uic, QtCore

import rospy, rostopic
from playground.msg import GameState

from matplotlib import pyplot as plt
from matplotlib import animation

import numpy as np

class UI(object):
    """docstring for UI"""
    def __init__(self, ui):
        super(UI, self).__init__()

        # Buttons
        self.btn_toggle_spacebar = ui.btnToggleSpacebar
        self.btn_us_goal = ui.btnUsGoal
        self.btn_them_goal = ui.btnThemGoal

        # Radio Buttons
        self.radio_one_v_one = ui.radOne
        self.radio_two_v_two = ui.radTwo

        # Labels
        self.lbl_us_score = ui.lblUsScore
        self.lbl_them_score = ui.lblThemScore

class GameState(object):
    def __init__(self, ui):
        super(Dialog, self).__init__()

        # Load and initialize the UI
        self.ui = UI(ui, ally)

        # Connect ROS things
        rospy.Subscriber('/game_state', GameState, self._handle_game_state)
        self.pub_game_state = rospy.Publisher('/game_state', GameState, queue_size=10)

        # Connect to Qt buttons
        self.ui.btn_toggle_spacebar.clicked.connect(self._btn_toggle_spacebar)
        self.ui.btn_us_goal.clicked.connect(self._btn_us_goal)
        self.ui.btn_them_goal.clicked.connect(self._btn_them_goal)

        # Connect to Qt Radio Buttons
        # self.ui.radio_one_v_one.toggled.connect()
        # self.ui.radio_two_v_two.toggled.connect()

        # Initialize Game State
        self.game_state = {
            'play': False,
            'one_v_one': False,
            'us_score': 0,
            'them_score': 0
        }

    # =========================================================================
    # ROS Event Callbacks (subscribers)
    # =========================================================================

    def _handle_game_state(self, msg):
        self.game_state['play'] = msg.play
        self.game_state['one_v_one'] = msg.one_v_one

        if msg.one_v_one:
            self.ui.radio_one_v_one.setChecked()
        else:
            self.ui.radio_two_v_two.setChecked()

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

        msg = GameState()
        msg.play = self.game_state['play']
        msg.one_v_one = self.game_state['one_v_one']
        msg.usgoal = False
        msg.themgoal = False
        self.pub_game_state(msg)

    def _btn_us_goal(self):
        self._increment_our_score()

        msg = GameState()
        msg.play = self.game_state['play']
        msg.one_v_one = self.game_state['one_v_one']
        msg.usgoal = True
        msg.themgoal = False
        self.pub_game_state(msg)

    def _btn_them_goal(self):
        self._increment_their_score()

        msg = GameState()
        msg.play = self.game_state['play']
        msg.one_v_one = self.game_state['one_v_one']
        msg.usgoal = False
        msg.themgoal = True
        self.pub_game_state(msg)

    # =========================================================================
    # Other
    # =========================================================================

    def _increment_our_score(self):
        self.game_state['us_score'] += 1
        self.ui.lbl_us_score.setText(self.game_state['us_score'])

    def _increment_their_score(self):
        self.game_state['them_score'] += 1
        self.ui.lbl_them_score.setText(self.game_state['them_score'])