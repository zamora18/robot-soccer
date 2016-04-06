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

    def update_state(self, msg):
        self.xhat = msg.xhat
        self.yhat = msg.yhat
        self.thetahat = msg.thetahat
        self.xhat_future = msg.xhat_future
        self.yhat_future = msg.yhat_future
        self.thetahat_future = msg.thetahat_future

    def get_2d_location(self):
        return (self.xhat, self.yhat)

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