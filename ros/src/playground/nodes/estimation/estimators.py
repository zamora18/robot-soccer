from collections import deque

import numpy as np

from filters import LowPassFilter, KalmanFilter

class BallEstimator(LowPassFilter):
    """BallEstimator
    Estimates the state of the ball using a filter. This estimator
    returns positions as well as velocities.

    State Definition:
        x = (x y)'
    """
    def __init__(self):

        T_ctrl = 1/100.0
        alpha = 0.75
        tau = 0.075
        update_type = LowPassFilter.UPDATE_SIMPLE
        N = 2

        super(BallEstimator, self).__init__(T_ctrl, alpha, tau, update_type, N)

    def _deal_with_bounces(self):
        """Deal with bounces
        """
        p = [self.position.getA()[0][0], self.position.getA()[0][1]]
        v = [self.velocity.getA()[0][0], self.velocity.getA()[0][1]]

        # check for bounce off end walls
        if  abs(p[0]) >= 3.68/2: # field length / 2
            v[0] =- v[0]

        # check for bounce off side walls
        if  abs(p[1]) >=  3.68/2: # field length / 2
            v[1] =- v[1]


class RobotEstimatorKF(KalmanFilter):
    """RobotEstimatorKF
    Uses a Kalman Filter to estimate the robot's state. This implementation
    depends on knowing the vel_cmds, so this is used for robots on our team
    as we only know the vel_cmds that we are sending out (i.e., we don't
    know our opponents vel_cmds... Unless we eavesdrop?).

    Here we use a constant velocity model. Therefore, our states contain
    positions only, and this estimator is primarily for smoothing noisy
    camera data and for dealing with camera latency. It may not be useful
    if vision threshold is done well and there is very low latency.

    State Definition:
        x = (x y theta)'
    """
    def __init__(self):

        # Update type
        update_type = self.UPDATE_SIMPLE

        # Camera latency and periods
        T_ctrl = 1/100.0
        T_cam = 1/30.0
        cam_latency = 130E-3 # seconds

        # Kalman filter model parameters
        N = 3
        O = np.zeros((N,N))
        I = np.eye(N,N)

        # A matrix (NxN)
        A = np.matrix( O )

        # B matrix
        B = np.matrix( I )

        # C matrix
        C = np.matrix( I )

        # Noise statistics (Q is process noise; R is sensor covariance)
        Q = np.matrix(np.diag([(.5E-2)**2, (.5E-2)**2, (5*np.pi/180)**2]))
        R = np.matrix(np.diag([0.001**2, 0.001**2, (1*np.pi/180)**2]))

        super(RobotEstimatorKF, self).__init__(update_type, T_ctrl, T_cam, cam_latency, A, B, C, Q, R)

class RobotEstimatorLPF(LowPassFilter):
    """RobotEstimatorLPF
    Estimates the state of the robot using a low-pass filter.
    This estimator returns positions as well as velocities.

    State Definition:
        x = (x y theta)'
    """
    def __init__(self):

        T_ctrl = 1/100.0
        alpha = 0.9
        tau = 0.075
        update_type = LowPassFilter.UPDATE_SIMPLE
        N = 3

        super(RobotEstimatorLPF, self).__init__(T_ctrl, alpha, tau, update_type, N)


class OpponentEstimator(KalmanFilter):
    """RobotEstimator
    Uses a Kalman Filter to estimate the opponent robot's state.

    Here we use a constant jerk model.

    State Definition:
        x = (p pdot pddot)

        where p = (x y theta) and a 'dot' or 'double dot (ddot)' implies time
        derivative. Thus, we estimate position, velocity and acceleration.
    """
    def __init__(self):

        # Update type
        update_type = self.UPDATE_SIMPLE

        # Camera latency and periods
        T_ctrl = 1/100.0
        T_cam = 1/30.0
        cam_latency = 130E-3 # seconds

        # Kalman filter model parameters
        N = 3
        O = np.zeros((N,N))
        I = np.eye(N,N)

        # A matrix (NxN)
        row1 = np.concatenate(( O, I, O ), axis=1)
        row2 = np.concatenate(( O, O, I ), axis=1)
        row3 = np.concatenate(( O, O, O ), axis=1)
        mat = np.concatenate(( row1, row2, row3 ))
        A = np.matrix(mat)

        # B matrix
        B = np.matrix( O )

        # C matrix
        mat = np.concatenate(( I, O, O ), axis=1)
        C = np.matrix(mat)

        # Noise statistics (Q is process noise; R is sensor covariance)
        Q = np.matrix(np.diag([(.5E-2)**2, (.5E-2)**2, (5*np.pi/180)**2]))
        R = np.matrix(np.diag([0.01**2, 0.01**2, (2*np.pi/180)**2]))

        super(RobotEstimator, self).__init__(T_ctrl, T_cam, cam_latency, A, B, C, Q, R)


def _mdiv(a, b):
    """Matrix Divide
    ignore division by 0

    Example: _mdiv( [-1, 0, 1], 0 ) -> [0, 0, 0]

    Is this element-wise or not? Should it be? or not?

    """
    with np.errstate(divide='ignore', invalid='ignore'):
        c = np.true_divide(a,b)
        c[c == np.inf] = 0
        c = np.nan_to_num(c)

    return c
