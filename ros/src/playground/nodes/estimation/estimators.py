from collections import deque

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

    def get_velocities(self):
        """Get Velocities
        """
        vx = self.velocity.getA()[0][0]
        vy = self.velocity.getA()[0][1]

        # return the current velocity estimate
        return (vx, vy)

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
    def __init__(self):
        super(RobotEstimator, self).__init__()

        self.update_type = 'FIXED_CAMERA_DELAY'

        # Sample rate
        self.T_ctrl = 1/100.0
        self.T_cam = 1/30.0

        # Initialize filter's persistent variables
        self.xhat = np.matrix([0, 0, 0])
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
        self.Q = np.matrix(np.diag([1**2, 1**2, (2*np.pi/180)**2]))
        self.R = np.matrix(np.diag([0.01**2, 0.01**2, (2*np.pi/180)**2]))


    def update(self, vel_cmd, measurement=None):
        """Update
        """
        
        # Convert incoming vel_cmds to a matrix
        vel_cmd = np.matrix(vel_cmd)

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
        y_pred = self.xhat

        # Kalman gain
        L = self.S/(self.R+self.S)

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

        y = measurement
        y_pred = self.xhat_d1

        # Kalman gain
        import ipdb; ipdb.set_trace()
        L = self.S_d1/(self.R+self.S_d1)

        # Update estimation error covariance
        I = np.matrix(np.eye( 3 ))
        self.S_d1 = (I - L)*self.S_d1

        # Correct state estimate
        self.xhat_d1 = self.xhat_d1 + L*(y-y_pred)

        # Propagate up to current point
        N = 10
        for i in xrange(N*self.discrete_delays):
            # Basically, this allows us to use the given old vel_cmd N times
            idx = int(np.ceil(i/N))
            self.xhat_d1 = self.xhat_d1 + (T_ctrl/N)*self.delayed_vel_cmds[idx]
            self.S_d1 = self.S_d1 + (T_ctrl/N)*self.Q

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
