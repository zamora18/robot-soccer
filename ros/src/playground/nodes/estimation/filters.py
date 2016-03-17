import numpy as np

from collections import deque

class LowPassFilter(object):
    """LowPassFilter
    Uses a weighted average low-pass filter with parameter alpha to
    update position. Then, velocity is estimated by taking a 
    dirty-derivative of the position. There are schemes to do the following:

    1. Estimate state
    2. Estimate state, compensating for camera delay
    3. Estimate state, compensating for camera delay and bounces

    The scheme is chosen via the update_type variable in __init__().
    """
    UPDATE_SIMPLE = 0
    UPDATE_DELAY = 1
    UPDATE_BOUNCE = 2

    def __init__(self, T_ctrl, alpha, tau, update_type, N):
        """LowPassFilter init method

        Args:
            T_ctrl          - 
            alpha           - The exponential parameter of the filter.
                              The higher the alpha, the more you trust
                              your own model, and not the new input.
            tau             - dirty-derivative bandwidth. Small numbers
                              allow derivatives to be more loose; i.e.,
                              0.001 creates lots of noise in velocity.
            update_type     - 
            N               - 
        """
        super(LowPassFilter, self).__init__()

        # Get the LPF update type
        self.update_type = update_type

        # Control loop period
        self.T_ctrl = T_ctrl
        
        # define alpha, used for the low-pass filter
        self.alpha = alpha

        # Dirty-derivative bandwidth
        self.tau = tau

        # number of states
        self.N = N

        # Persistent variables (init better?)
        self.position = np.matrix(np.zeros(N))
        self.position_d1 = self.position
        self.velocity = np.matrix(np.zeros(N))
        self.measurement_d1 = self.position

    def get_velocities(self):
        """Get Velocities
        """

        # return the current velocity estimate
        return self._unpack_states(self.velocity)

    def update(self, Ts, measurement=None):
        """Update
        """

        if self.update_type == self.UPDATE_SIMPLE:
            self._update_simple(Ts, measurement)

        elif self.update_type == self.UPDATE_DELAY:
            self._update_delayed(Ts, measurement)

        elif self.update_type == self.UPDATE_BOUNCE:
            self._update_delayed_bounce(Ts, measurement)

        # return the estimated position states
        return self._unpack_states(self.position)


    def predict(self, t):
        """Predict

        t:  how far (seconds) into the future to push forward the diff eq

        returns where (xhat, yhat) will be in t seconds
        """

        # Start with our current position
        pos = self.position

        # Figure out how many steps of t = T_ctrl to make
        N = int(np.floor(t/self.T_ctrl))

        # Step into the future N times
        for i in xrange(N):
            pos = pos + (self.T_ctrl)*self.velocity

        # return our future position at time t
        return self._unpack_states(pos)


    def _update_simple(self, Ts, measurement=None):
        """Update Simple
        Estimates states, with no regard for camera latencies

        This matches case 1 of the MATLAB utility_lpf_ball() function.

        Ts:             time since last camera measurement. Only
                        useful on valid camera measurements.
        measurement:    positions, from the vision system

        If measurement is None, then there was no camera update.
        So don't do anything.
        """
        # We need to know when to correct our predictions and when to
        # only push forward our predictor.
        measurement_received = (measurement.count(None) == 0)

        if measurement_received: # correct
            
            # Create a row vector (np.matrix) for easy computation
            measurement = np.matrix(measurement)

            # low pass filter position 
            self.position = self.alpha*self.position + (1-self.alpha)*measurement

            # update velocity using position measurement
            self._update_velocity(measurement, Ts)


    def _update_delayed(self, Ts, measurement=None):
        """Update Delayed
        Estimates states while compensating for variable camera delay.

        This matches case 2 of the MATLAB utility_lpf_ball() function.

        Ts:             time since last camera measurement. Only
                        useful on valid camera measurements.
        measurement:    positions, from the vision system

        If measurement is not None, then the system knows
        that there is a new camera measurement, so the correction
        branch is executed.
        """
        # We need to know when to correct our predictions and when to
        # only push forward our predictor.
        measurement_received = (measurement.count(None) == 0)

        if measurement_received: # correction
            
            # Create a row vector (np.matrix) for easy computation
            measurement = np.matrix(measurement)

            # low pass filter position 
            self.position_d1 = self.alpha*self.position_d1 + (1-self.alpha)*measurement

            # update velocity using position measurement
            self._update_velocity(measurement, Ts)

            # Reset position to where it was when this measurement was valid
            self.position = self.position_d1

            # And propagate up to current location
            self.position = np.matrix(self.predict(Ts))

        else: # prediction
            # propagate prediction ahead one control sample time
            self.position = np.matrix(self.predict(self.T_ctrl))


    def _updated_delayed_bounce(Ts, ball_x=None, ball_y=None):
        """Update Delayed
        Estimates states while compensating for variable camera delay
        and wall bounces (but not robot bounces).

        This matches case 3 of the MATLAB utility_lpf_ball() function.

        Ts:             time since last camera measurement. Only
                        useful on valid camera measurements.
        measurement:    positions, from the vision system

        If measurement is not None, then the system knows
        that there is a new camera measurement, so the correction
        branch is executed.
        """
        pass


    def _update_velocity(self, measurement, Ts):
        """Calculate Velocity
        Using the Tustin Approximation (Bilinear Transform), compute
        a discrete dirty-derivative of the position measurement.

        measurement:    current position measurement from camera
        Ts:             camera sample rate
        """

        # Update the current velocity
        self.velocity = (2*self.tau-Ts)/(2*self.tau+Ts)*self.velocity + \
                    2/(2*self.tau+Ts)*(measurement-self.measurement_d1)

        # Save the current measurement for the next time
        # I need to calculate what the velocity was.
        self.measurement_d1 = measurement

    def _unpack_states(self, mat):
        """Unpack States

        Expects a row vector (np.matrix) of length self.N
        and returns a list of elements
        """
        xhat = []
        for i in xrange(self.N):
            xhat.append(mat.getA()[0][i])

        return xhat

    def _deal_with_bounces(self):
        """Deal with bounces
        This should be implemented by subclasses of LPF. For the ball,
        this could deal with wall bounces and bounces off of robots.
        """
        pass


class KalmanFilter(object):
    """KalmanFilter
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

    we get lots of noise because of theta. Because of statistics, the 
    KF thinks that noisy theta means noisy x and y. So try separating it
    out, or unwrapping theta. Theta noisy comes from the fact that at 0,
    the camera sends that theta is 0 and 360. Unwrap values coming in,
    and wrap values going out.

    """

    UPDATE_SIMPLE = 0
    UPDATE_FIXED_DELAY = 1

    def __init__(self, update_type, T_ctrl, T_cam, cam_latency, A, B, C, Q, R):
        super(KalmanFilter, self).__init__()

        self.update_type = update_type

        # Sample period
        self.T_ctrl = T_ctrl
        self.T_cam = T_cam

        # Number of states
        self.N = len(A)

        # Initialize filter's persistent variables
        self.xhat = np.matrix(np.zeros( self.N )).T
        self.xhat_d1 = self.xhat
        self.S = np.matrix(np.diag(np.zeros( self.N )))
        self.S_d1 = self.S

        # In one camera frame, how many control cycles happen?
        control_cycles_per_frame = int(np.ceil(self.T_cam/self.T_ctrl))

        # How many frames behind is the camera?
        # It was reported that our camera was having 100-150ms of latency
        # through the network (cam --> usb --> camserver --> network --> camserver)
        # But I'm not sure that it is true (try sending timestamped ROS messages)
        camera_frame_delay = int(np.ceil(cam_latency/self.T_cam))

        # The ratio of when there is a new camera measurement vs when
        # this estimator's update() function will be called
        self.discrete_delays = control_cycles_per_frame*camera_frame_delay

        # Initialize delayed incoming velocity commands
        self.delayed_vel_cmds = deque()
        for i in xrange(self.discrete_delays):
            self.delayed_vel_cmds.append(np.matrix([0, 0, 0]))

        # Kalman filter model parameters
        self.A = A
        self.B = B
        self.C = C

        # Noise statistics
        self.Q = Q
        self.R = R


    def update(self, measurement=None, vel_cmd=None):
        """Update
        """
        
        # Convert incoming vel_cmds to a column vector
        vel_cmd = np.matrix(vel_cmd).T if vel_cmd.count(None) == 0 else None

        # Prediction step between measurements
        #   This could probably be optomized out for
        #   FIXED_CAMERA_DELAY on non-measurements
        N = 10
        time_step = self.T_ctrl/N
        u = 0 if vel_cmd is None else vel_cmd
        for i in xrange(N):
            self.xhat = self.xhat + time_step*( self.A*self.xhat + self.B*u )
            self.S = self.S + time_step*(  self.A*self.S + self.S*self.A.T + self.Q )

        # correction step at measurement
        if measurement.count(None) == 0:
            # Only update when a camera measurement came in

            if self.update_type == self.UPDATE_SIMPLE:
                self._update_simple(measurement)

            elif self.update_type == self.UPDATE_FIXED_DELAY:
                self._update_delayed(vel_cmd, measurement)

        return self._unpack_states(self.xhat)

    def predict(self):
        """Predict
        """
        pass
        

    def _update_simple(self, measurement):
        """Update Simple
        """
        y = np.matrix(measurement).T
        y_pred = self.C*self.xhat

        # Kalman gain
        L = _mdiv((self.S*self.C.T), (self.R + self.C*self.S*self.C.T))

        # Update estimation error covariance
        I = np.matrix(np.eye( self.N ))
        self.S = (I - L*self.C)*self.S

        # Correct state estimate
        self.xhat = self.xhat + L*(y-y_pred)


    def _update_delayed(self, measurement, vel_cmd=None):
        """Update Delayed

        This function is case 2 of Dr. Beard's utility_kalman_filter_opponent.
        It takes in to account fixed camera delay.

        Relevant equations are: (from Robot Soccer Notes)
            - the four equations describing how to update the Kalman gain,
                error covariance, and state estimation at the measurement
                of the sensor. (D. 16)

        measurement is expected as a tuple: (x, y, theta)
        """
        # Shift old velocity commands
        if vel_cmd is not None:
            self._shift_delayed_vel_cmds(vel_cmd)

        y = np.matrix(measurement).T
        y_pred = self.C*self.xhat_d1

        # Kalman gain
        L = _mdiv(self.S_d1*self.C.T), (self.R + self.C*self.S_d1*self.C.T)

        # Update estimation error covariance
        I = np.matrix(np.eye( self.N ))
        self.S_d1 = (I - L*self.C)*self.S_d1

        # Correct state estimate
        self.xhat_d1 = self.xhat_d1 + L*(y-y_pred)

        # Propagate up to current point
        N = 10
        time_step = T_ctrl/N
        for i in xrange(N*self.discrete_delays):
            # Basically, this allows us to use the given old vel_cmd N times
            idx = int(np.floor(np.true_divide(i,N)))
            u = 0 if self.delayed_vel_cmds[i] is None else self.delayed_vel_cmds[i]

            self.xhat_d1 = self.xhat_d1 + time_step*( self.A*self.xhat_d1 + self.B*u)
            self.S_d1 = self.S_d1 + time_step*( self.A*self.S_d1 + self.S_d1*self.A.T + self.Q )

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


    def _unpack_states(self, mat):
        """Unpack States

        Expects a row vector (np.matrix) of length self.N
        and returns a list of elements
        """
        xhat = []
        for i in xrange(self.N):
            xhat.append(mat.getA()[i][0])

        return xhat


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
