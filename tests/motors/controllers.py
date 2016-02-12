class PID(object):
    """docstring for PID"""
    def __init__(self, kp, ki, kd, limit, tau, integrator_limit=0.05):
        super(PID, self).__init__()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.tau = tau
        self.integrator_limit = integrator_limit
        

    def update(self, x_c, x, Ts):
        # Declare persistent/static variables
        if "integrator" not in PID.__dict__: PID.integrator = 0
        if "xdot" not in PID.__dict__: PID.xdot = 0
        if "error_d1" not in PID.__dict__: PID.error_d1 = 0
        if "x_d1" not in PID.__dict__: PID.x_d1 = 0

        # compute the error
        error = x_c - x

        # update derivative of x
        PID.xdot = _tustin_derivative(tau, Ts, PID.xdot, PID.x_d1, x)

        # update integral of error
        if ki and abs(PID.xdot)<self.integrator_limit:
            PID.integrator = _tustin_integral(Ts, PID.integrator, error,
                                                        PID.error_d1)

        # update delayed variables for next time
        PID.error_d1 = error
        PID.x_d1 = x

        # compute the PID control signal
        u_unsat = kp*error + ki*PID.integrator - kd*PID.xdot;
        u = u_unsat
    #    u = _sat(u_unsat, limit)

        # more integrator anti-windup
        if ki:
            PID.integrator = PID.integrator + Ts/ki*(u-u_unsat)

        return u


    def _sat(self, val, limit):
        out = 0

        if val > limit:
            out = limit
        elif val < limit:
            out = -limit
        else:
            out = val

        return out

    def _tustin_derivative(self, tau, Ts, xdot, x_d1, x):
        return (2*tau-Ts)/(2*tau+Ts)*xdot + 2/(2*tau+Ts)*(x-x_d1)

    def _tustin_integral(self, Ts, integrator, x, x_d1):
        return integrator + (Ts/2)*(x+x_d1)