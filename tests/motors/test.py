import roboclaw as rc

roboclaw = rc.Roboclaw(128, '/dev/ttySAC0/', 2400)
roboclaw.drive_forward_m1(64)
