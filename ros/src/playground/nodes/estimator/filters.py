import numpy as np

class LowpassFilter(object):
    """Lowpass Filter
    

    alpha: asdf
    """
    def __init__(self, alpha, tau):
        super(LowpassFilter, self).__init__()
        self.alpha = alpha
        self.tau = tau

        # Persistent
        self.position = np.matrix([0, 0])
        self.position_d1 = self.position
        self.velocity = np.matrix([0, 0])
        self.old_position_measurement = self.position
        

    def filter(self, ball_x, ball_y, Ts):
        """Filter

        This is case 2, which compensates for fixed camera delay
        """
        Ts_control = 1/100.0

        vision_ball_position = np.matrix([ball_x, ball_y])

        if 1: # correct
            # low pass filter position 
            self.position_d1 = self.alpha*self.position_d1 + (1-self.alpha)*vision_ball_position

            # compute velocity by dirty derivative of position
            self.velocity = self._tustin_derivative(vision_ball_position, Ts)
            self.old_position_measurement = vision_ball_position

            # propagate upto current location
            N = int(np.floor(Ts/Ts_control))
            for i in range(N):
                self.position_d1 = self.position_d1 + Ts*self.velocity

            self.position = self.position_d1

        else: # prediction
            # propagate prediction ahead one control sample time
            self.position = self.position + Ts_control*self.velocity


        import ipdb; ipdb.set_trace()

        xhat = self.position.getA()[0][0]
        yhat = self.position.getA()[0][1]
        
        return (xhat, yhat)


    def _tustin_derivative(self, vision_ball_position, Ts):
        """Tustin Derivative
        Using the Tustin Approximation (Bilinear Transform), compute
        a discrete dirty-derivative.

        Ts: camera sample rate
        """
        return (2*self.tau-Ts)/(2*self.tau+Ts)*self.velocity + 2/(2*self.tau+Ts)*(vision_ball_position-self.old_position_measurement)

    def _wall_bounce(self):
        """Wall Bounce
        Lowpass filter ball position - differentiate for velocity
        """
        p = [self.position.getA()[0][0], self.position.getA()[0][1]]
        v = [self.velocity.getA()[0][0], self.velocity.getA()[0][1]]

        # check for bounce off end walls
        if  abs(p[0]) >= 3.68/2: # field length / 2
            v[0] =- v[0]

        # check for bounce off side walls
        if  abs(p[1]) >=  3.68/2: # field length / 2
            v[1] =- v[1]