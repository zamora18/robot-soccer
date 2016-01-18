import roboclaw as rc

roboclaw = rc.RoboClaw(128, '/dev/ttySAC0', 2400)


def forward(speed):
	roboclaw.drive_forward_m1(speed)
	roboclaw.drive_forward_m2(speed)

def stop():
	roboclaw.drive_forward_m1(0)
	roboclaw.drive_forward_m2(0)

def readPIDQ():
    print roboclaw.read_m1_velocity_PID_QPPS()