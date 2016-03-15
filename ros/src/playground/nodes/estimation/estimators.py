from collections import deque

import numpy as np

from filters import LowPassFilter

class BallEstimator(LowPassFilter):
    """BallEstimator
    Uses a weighted average low-pass filter with parameter alpha to
    update position. Then, velocity is estimated by taking a 
    dirty-derivative of the position. There are schemes to do the following:

    1. Estimate ball state
    2. Estimate ball state, compensating for camera delay
    3. Estimate ball state, compensating for camera delay and wall bounces

    The scheme is chosen via the self.update_type variable.
    """
    def __init__(self):

        T_ctrl = 1/100.0
        alpha = 0.75
        tau = 0.075
        update_type = LowPassFilter.UPDATE_SIMPLE
        N = 2

        super(BallEstimator, self).__init__(T_ctrl, alpha, tau, update_type, N)

    def _wall_bounce(self):
        """Wall Bounce
        Untested.
        """
        p = [self.position.getA()[0][0], self.position.getA()[0][1]]
        v = [self.velocity.getA()[0][0], self.velocity.getA()[0][1]]

        # check for bounce off end walls
        if  abs(p[0]) >= 3.68/2: # field length / 2
            v[0] =- v[0]

        # check for bounce off side walls
        if  abs(p[1]) >=  3.68/2: # field length / 2
            v[1] =- v[1]



class RobotEstimator(object):
    """RobotEstimator
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
        super(RobotEstimator, self).__init__()

        self.update_type = 'SIMPLE' #'FIXED_CAMERA_DELAY'

        # Sample rate
        self.T_ctrl = 1/100.0
        self.T_cam = 1/30.0

        # Initialize filter's persistent variables
        self.xhat = np.matrix([0, 0, 0]).T
        self.xhat_d1 = self.xhat
        self.S = np.matrix(np.diag([0, 0, 0]))  # should probably use arrays, 
                                                # but I just learned that and
                                                # want to be consistent
        self.S_d1 = self.S

        # In one camera frame, how many control cycles happen?
        control_cycles_per_frame = int(np.ceil(self.T_cam/self.T_ctrl))

        # How many frames behind is the camera?
        # It was reported that our camera was having 100-150ms of latency
        # through the network (cam --> usb --> camserver --> network --> camserver)
        # But I'm not sure that it is true (try sending timestamped ROS messages)
        camera_frame_delay = int(np.ceil(130E-3/self.T_cam))

        # The ratio of when there is a new camera measurement vs when
        # this estimator's update() function will be called
        self.discrete_delays = control_cycles_per_frame*camera_frame_delay

        # Initialize delayed incoming velocity commands
        self.delayed_vel_cmds = deque()
        for i in xrange(self.discrete_delays):
            self.delayed_vel_cmds.append(np.matrix([0, 0, 0]))

        # Noise statistics
        self.Q = np.matrix(np.diag([(5E-2)**2, (5E-2)**2, (10*np.pi/180)**2]))
        self.R = np.matrix(np.diag([0.01**2, 0.01**2, (2*np.pi/180)**2]))


    def update(self, vel_cmd, measurement=None):
        """Update
        """
        
        # Convert incoming vel_cmds to a column vector
        vel_cmd = np.matrix(vel_cmd).T

        # Prediction step between measurements
        #   This could probably be optomized out for
        #   FIXED_CAMERA_DELAY on non-measurements
        N = 10
        for i in xrange(N):
            self.xhat = self.xhat + (self.T_ctrl/N)*vel_cmd
            self.S = self.S + (self.T_ctrl/N)*self.Q

        # correction step at measurement
        if measurement.count(None) == 0:
            # Only update when a camera measurement came in

            if self.update_type == 'SIMPLE':
                self._update_simple(measurement)

            elif self.update_type == 'FIXED_CAMERA_DELAY':
                self._update_delayed(vel_cmd, measurement)

        # The naming is unfortunate, but self.xhat is the state estimation
        # while the local xhat is the x-position estimate
        xhat = self.xhat.getA()[0][0]
        yhat = self.xhat.getA()[1][0]
        thetahat = self.xhat.getA()[2][0]
        
        return (xhat, yhat, thetahat)

    def predict(self):
        """Predict
        """
        pass
        

    def _update_simple(self, measurement):
        """Update Simple
        """
        y = np.matrix(measurement).T
        y_pred = self.xhat

        # Kalman gain
        L = _mdiv(self.S, (self.R+self.S))

        # Update estimation error covariance
        I = np.matrix(np.eye(3))
        self.S = (I - L)*self.S

        # Correct state estimate
        self.xhat = self.xhat + L*(y-y_pred)


    def _update_delayed(self, vel_cmd, measurement):
        """Update Delayed
        """

        # Shift old velocity commands
        self._shift_delayed_vel_cmds(vel_cmd)

        y = np.matrix(measurement).T
        y_pred = self.xhat_d1

        # Kalman gain
        L = _mdiv(self.S_d1, (self.R+self.S_d1))

        # Update estimation error covariance
        I = np.matrix(np.eye( 3 ))
        self.S_d1 = (I - L)*self.S_d1

        # Correct state estimate
        self.xhat_d1 = self.xhat_d1 + L*(y-y_pred)

        # Propagate up to current point
        N = 10
        for i in xrange(N*self.discrete_delays):
            # Basically, this allows us to use the given old vel_cmd N times
            idx = int(np.floor(np.true_divide(i,N)))
            self.xhat_d1 = self.xhat_d1 + (self.T_ctrl/N)*self.delayed_vel_cmds[idx]
            self.S_d1 = self.S_d1 + (self.T_ctrl/N)*self.Q

        # Update current estimate
        self.xhat = self.xhat_d1
        self.S = self.S_d1

    def _shift_delayed_vel_cmds(self, vel_cmd):
        """Shift Delayed Velocity Commands
        """

        # Move everything to the left once
        self.delayed_vel_cmds.rotate(-1)

        # Because rotate moves the first element to the end, remove it
        self.delayed_vel_cmds.pop()

        # append the new velocity command, as an np.matrix
        self.delayed_vel_cmds.append(np.matrix(vel_cmd))




class OpponentEstimator(object):
    """RobotEstimator
    Uses a Kalman Filter to estimate the opponent robot's state.

    Here we use a constant jerk model.

    State Definition:
        x = (p pdot pddot)

        where p = (x y theta) and a 'dot' or 'double dot (ddot)' implies time
        derivative. Thus, we estimate position, velocity and acceleration.
    """
    def __init__(self, arg):
        super(RobotEstimator, self).__init__()

        self.update_type = 'FIXED_CAMERA_DELAY'

        # Sample rate
        self.T_ctrl = 1/100.0
        self.T_cam = 1/30.0

        # Initialize filter's persistent variables
        self.xhat = np.matrix([0, 0, 0]*3)
        self.xhat_d1 = self.xhat
        self.S = np.matrix(np.diag([0, 0, 0]*3))
        self.S_d1 = self.S

        # In one camera frame, how many control cycles happen?
        control_cycles_per_frame = int(np.ceil(self.T_cam/self.T_ctrl))

        # How many frames behind is the camera?
        # It was reported that our camera was having 100-150ms of latency
        # through the network (cam --> usb --> camserver --> network --> camserver)
        # But I'm not sure that it is true (try sending timestamped ROS messages)
        camera_frame_delay = int(np.ceil(130E-3/self.T_cam))

        # The ratio of when there is a new camera measurement vs when
        # this estimator's update() function will be called
        self.discrete_delays = control_cycles_per_frame*camera_frame_delay

        # Kalman filter model parameters
        z3 = np.zeros((3,3))
        i3 = np.eye(3,3)
        row1 = np.concatenate(( z3, i3, z3 ), axis=1)
        row2 = np.concatenate(( z3, z3, i3 ), axis=1)
        row3 = np.concatenate(( z3, z3, z3 ), axis=1)
        mat = np.concatenate(( row1, row2, row3 ))
        self.A = np.matrix(mat)

        self.B = np.matrix(np.zeros( (len(self.A),len(self.A)) ))

        mat = np.concatenate(( i3, z3, z3), axis=1)
        self.C = np.matrix(mat)

        # Noise statistics
        self.Q = np.matrix(np.diag([1**2, 1**2, (2*np.pi/180)**2]))
        self.R = np.matrix(np.diag([0.01**2, 0.01**2, (2*np.pi/180)**2]))


    def update(self, measurement=None):
        """Update
        """
       
        # Prediction step between measurements
        #   This could probably be optomized out for
        #   FIXED_CAMERA_DELAY on non-measurements
        N = 10
        time_step = self.T_ctrl/N
        u = 0
        for i in xrange(N):
            self.xhat = self.xhat + time_step*( self.A*self.xhat + self.B*u )
            self.S = self.S + time_step*(  self.A*self.S + self.S*self.A.T + self.Q )

        # correction step at measurement
        if measurement is not None:
            # Only update when a camera measurement came in

            if self.update_type == 'SIMPLE':
                self._update_simple(measurement)

            elif self.update_type == 'FIXED_CAMERA_DELAY':
                self._update_delayed(measurement)

        # The naming is unfortunate, but self.xhat is the state estimation
        # while the local xhat is the x-position estimate
        xhat = self.xhat.getA()[0][0]
        yhat = self.xhat.getA()[0][1]
        thetahat = self.xhat.getA()[0][2]
        
        return (xhat, yhat, thetahat)

    def predict(self):
        """Predict
        """
        pass
        

    def _update_simple(self, measurement):
        """Update Simple
        """
        y = measurement
        y_pred = self.C*self.xhat

        # Kalman gain
        L = (self.S*self.C.T) / (self.R + self.C*self.S*self.C.T)

        # Update estimation error covariance
        I = np.matrix(np.eye( len(self.A) ))
        self.S = (I - L*self.C)*self.S

        # Correct state estimate
        self.xhat = self.xhat + L*(y-y_pred)


    def _update_delayed(self, measurement):
        """Update Delayed

        This function is case 2 of Dr. Beard's utility_kalman_filter_opponent.
        It takes in to account fixed camera delay.

        Relevant equations are: (from Robot Soccer Notes)
            - the four equations describing how to update the Kalman gain,
                error covariance, and state estimation at the measurement
                of the sensor. (D. 16)

        measurement is expected as a tuple: (x, y, theta)
        """
        y = measurement
        y_pred = self.C*self.xhat_d1

        # Kalman gain
        L = (self.S_d1*self.C.T) / (self.R + self.C*self.S_d1*self.C.T)

        # Update estimation error covariance
        I = np.matrix(np.eye( len(self.A) ))
        self.S_d1 = (I - L*self.C)*self.S_d1

        # Correct state estimate
        self.xhat_d1 = self.xhat_d1 + L*(y-y_pred)

        # Propagate up to current point
        N = 10
        time_step = T_ctrl/N
        for i in xrange(N*self.discrete_delays):
            self.xhat_d1 = self.xhat_d1 + time_step*( self.A*self.xhat_d1 )
            self.S_d1 = self.S_d1 + time_step*( self.A*self.S_d1 + self.S_d1*self.A.T + self.Q )

        # Update current estimate
        self.xhat = self.xhat_d1
        self.S = self.S_d1

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
