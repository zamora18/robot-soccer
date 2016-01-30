import rcv3.roboclaw as r

r.Open('/dev/ttySAC0', 38400)

def readPIDQ():
	pid

def spin(speed):
	r.SpeedM1(0x80, speed)
	r.SpeedM2(0x80, speed)
	r.SpeedM1(0x81, speed)

def forward(speed):
	r.SpeedM1(0x80, speed)
	r.SpeedM2(0x80, speed)

def stop():
	r.ForwardM1(0x80, 0)
	r.ForwardM2(0x80, 0)
	r.ForwardM1(0x81, 0)
