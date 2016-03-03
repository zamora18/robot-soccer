import numpy as np

class BallEstimator(object):
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
        super(BallEstimator, self).__init__()

        self.update_type = 'SIMPLE' #'DELAYED_CAMERA'

        # Control loop rate
        self.f_ctrl = 100;
        self.T_ctrl = 1.0/self.f_ctrl
        
        # define alpha, used for the low-pass filter
        self.alpha = 0.3

        # Dirty-derivative bandwidth
        self.tau = 0.02

        # Persistent variables (init better?)
        self.position = np.matrix([0, 0])
        self.position_d1 = self.position
        self.velocity = np.matrix([0, 0])
        self.measurement_d1 = self.position

    def update(self, Ts, ball_x=None, ball_y=None):
        """Update
        """

        if self.update_type == 'SIMPLE':
            self._update_simple(Ts, ball_x, ball_y)

        elif self.update_type == 'DELAYED_CAMERA':
            self._update_delayed(Ts, ball_x, ball_y)

        elif self.update_type == 'DELAYED_CAMERA_WALL_BOUNCE':
            self._update_delayed_bounce(Ts, ball_x, ball_y)

        xhat = self.position.getA()[0][0]
        yhat = self.position.getA()[0][1]
        
        return (xhat, yhat)


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

        xhat = pos.getA()[0][0]
        yhat = pos.getA()[0][1]

        # return our future position at time t
        return (xhat, yhat)


    def _update_simple(self, Ts, ball_x=None, ball_y=None):
        """Update Simple
        Estimates states, with no regard for camera latencies

        This matches case 1 of the MATLAB utility_lpf_ball() function.

        Ts:         time since last camera measurement. Only useful on
                    valid camera measurements. See ball.
        ball_x:     x position of the ball, from the vision system
        ball_y:     y position of the ball, from the vision system

        If ball_x and ball_y are not None, then the system knows
        that there is a new camera measurement, so the estimate
        branch is executed.
        """
        # We need to know when to correct our predictions and when to
        # only push forward our predictor.
        measurement_received = ball_x is not None and ball_y is not None

        if measurement_received: # estimate
            
            # Create a vector for easy computation
            measurement = np.matrix([ball_x, ball_y])

            # low pass filter position 
            self.position = self.alpha*self.position + (1-self.alpha)*measurement

            # update ball's velocity using position measurement
            self._update_velocity(measurement, Ts)


    def _update_delayed(self, Ts, ball_x=None, ball_y=None):
        """Update Delayed
        Estimates states while compensating for variable camera delay.

        This matches case 2 of the MATLAB utility_lpf_ball() function.

        Ts:         time since last camera measurement. Only useful on
                    valid camera measurements. See ball.
        ball_x:     x position of the ball, from the vision system
        ball_y:     y position of the ball, from the vision system

        If ball_x and ball_y are not None, then the system knows
        that there is a new camera measurement, so the correction
        branch is executed.
        """
        # We need to know when to correct our predictions and when to
        # only push forward our predictor.
        measurement_received = ball_x is not None and ball_y is not None

        if measurement_received: # correction
            
            # Create a vector for easy computation
            measurement = np.matrix([ball_x, ball_y])

            # low pass filter position 
            self.position_d1 = self.alpha*self.position_d1 + (1-self.alpha)*measurement

            # update ball's velocity using position measurement
            self._update_velocity(measurement, Ts)

            # Reset ball's position to where it was when this measurement was valid
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

        Ts:         time since last camera measurement. Only useful on
                    valid camera measurements. See ball.
        ball_x:     x position of the ball, from the vision system
        ball_y:     y position of the ball, from the vision system

        If ball_x and ball_y are not None, then the system knows
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

        # Update the ball's current velocity
        self.velocity = (2*self.tau-Ts)/(2*self.tau+Ts)*self.velocity + \
                    2/(2*self.tau+Ts)*(measurement-self.measurement_d1)

        # Save the current measurement for the next time
        # I need to calculate what the velocity was.
        self.measurement_d1 = measurement


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
        x = (x y theta)
    """
    def __init__(self, arg):
        super(RobotEstimator, self).__init__()

        self.update_type = 'DELAYED_CAMERA'

        # Initialize filter's persistent variables
        self.xhat = np.matrix([0, 0, 0])
        self.xhat_d1 = self.xhat
        self.S = np.matrix(np.diag([0, 0, 0]))  # should probably use arrays, 
                                                # but I just learned that and
                                                # want to be consistent
        self.S_d1 = self.S

        self.max_delay_idx = 1

        # fix this...
        self.vel_cmd_d1 = 0

        # Noise statistics
        self.Q = np.matrix(np.diag([1**2, 1**2, (2*pi/180)**2]))
        self.R = np.matrix(np.diag([0.01**2, 0.01**2, (2*pi/180)**2]))


    def update(self, vel_cmd, measurement=None):
        """Update
        """
       
        # Prediction step between measurements
        N = 10
        for i in xrange(N):
            self.xhat = self.xhat + (T_ctrl/N)*vel_cmd
            self.S = self.S + (T_ctrl/N)*self.Q

        # correction step at measurement
        if measurement is not None:
            # Only update when a camera measurement came in

            if self.update_type == 'SIMPLE':
                self._update_simple(Ts, measurement)

            elif self.update_type == 'DELAYED_CAMERA':
                self._update_delayed(Ts, measurement)

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
        

    def _update_simple(self, Ts, measurement):
        """Update Simple
        """
        y = measurement
        y_pred = self.xhat

        # Kalman gain
        L = self.S/(self.R+self.S)

        # Update estimation error covariance
        eye3 = np.matrix(np.eye(3))
        self.S = eye3 - L*self.S

        # Correct state estimate
        self.xhat = self.xhat + L*(y-y_pred)


    def _update_delayed(self, xhat=None, yhat=None):
        """Update Delayed
        """
        y = measurement
        y_pred = self.xhat_d1

        # Kalman gain
        L = self.S_d1/(self.R+self.S_d1)

        # Update estimation error covariance
        eye3 = np.matrix(np.eye(3))
        self.S_d1 = eye3 - L*self.S_d1

        # Correct state estimate
        self.xhat_d1 = self.xhat_d1 + L*(y-y_pred)

        # Propagate up to current point
        N = 10
        for i in xrange(N*self.max_delay_idx):
            self.xhat_d1 = self.xhat_d1 + (T_ctrl/N)*self.vel_cmd_d1 # fix this at the right index...
            self.S_d1 = self.S_d1 + (T_ctrl/N)*self.Q

        # Update current estimate
        self.xhat = self.xhat_d1
        self.S = self.S_d1
