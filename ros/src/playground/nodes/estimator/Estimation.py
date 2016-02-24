import numpy as np

from filters import LowpassFilter

ball_lpf = LowpassFilter(0.7, 0.02)

_set_point = (0, 0, 0)

velocities = (0, 0, 0)

def init():
    pass
