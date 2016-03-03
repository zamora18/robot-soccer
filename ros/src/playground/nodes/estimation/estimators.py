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

        self.update_type = 'DELAYED_CAMERA'

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
        N = int(np.floor(Ts/self.T_ctrl))

        # Step into the future N times
        for i in xrange(N):
            pos = pos + self.T_ctrl*self.velocity

        # return our future position at time t
        return pos


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
            self.position = self.predict(Ts)

        else: # prediction
            # propagate prediction ahead one control sample time
            self.position = self.predict(self.T_ctrl)


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
    """
    def __init__(self, arg):
        super(RobotEstimator, self).__init__()

        self.update_type = 'DELAYED_CAMERA'

        # Persistent variables (init better?)
        self.xhat = np.matrix([0, 0])
        self.xhat_d1 = self.xhat
        self.S = np.matrix([0, 0])
        self.S_d1 = self.S
        self.vel_cmd_d1 = 0


    def update(self, xhat=None, yhat=None):
        """Update
        """
        
        # Prediction step between measurements
        N = 10
        for i in xrange(N):
            pass

        # correction step at measurement
        if self.update_type == 'SIMPLE':
            self._update_simple(Ts, ball_x, ball_y)

        elif self.update_type == 'DELAYED_CAMERA':
            self._update_delayed(Ts, ball_x, ball_y)

    def predict(self):
        """Predict
        """
        pass
        

    def _update_simple(self, xhat=None, yhat=None):
        """Update Simple
        """
        pass


    def _update_delayed(self, xhat=None, yhat=None):
        """Update Delayed
        """
        pass