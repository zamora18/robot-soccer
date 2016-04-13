class Robot(object):
    """docstring for Robot"""
    def __init__(self, ally1=False, ally2=False):
        super(Robot, self).__init__()
        self.ally1 = ally1
        self.ally2 = ally2
        self.xhat = 0
        self.yhat = 0
        self.thetahat = 0
        self.xhat_future = 0
        self.yhat_future = 0
        self.thetahat_future = 0
        self.x_c = None
        self.y_c = None
        self.theta_c = None

    def update_state(self, msg):
        self.xhat = msg.xhat
        self.yhat = msg.yhat
        self.thetahat = msg.thetahat
        self.xhat_future = msg.xhat_future
        self.yhat_future = msg.yhat_future
        self.thetahat_future = msg.thetahat_future

    def update_desired(self, msg):
        self.x_c = msg.x
        self.y_c = msg.y
        self.theta_c = msg.theta

    def get_2d_location(self):
        return (self.xhat, self.yhat)

    def get_2d_desired(self):
        return (self.x_c, self.y_c)

    def has_a_desired_position(self):
        if self.x_c is None or self.y_c is None or self.theta_c is None:
            return False

        return True

class Ball(object):
    """docstring for Ball"""
    def __init__(self):
        super(Ball, self).__init__()
        self.xhat = 0
        self.yhat = 0
        self.xhat_future = 0
        self.yhat_future = 0

    def update_state(self, msg):
        self.xhat = msg.xhat
        self.yhat = msg.yhat
        self.xhat_future = msg.xhat_future
        self.yhat_future = msg.yhat_future

    def get_2d_location(self):
        return (self.xhat, self.yhat)