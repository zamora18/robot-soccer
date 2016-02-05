import rcv3.roboclaw as r
import time
import math

r.Open('/dev/ttySAC0', 38400)

def readPIDQ():
	PIDQ = []
	PIDQ.append(r.ReadM1VelocityPID(0x80))
	PIDQ.append(r.ReadM2VelocityPID(0x80))
	PIDQ.append(r.ReadM1VelocityPID(0x81))
	return PIDQ

def setAllPIDQ(p,i,d,q):
	r.SetM1VelocityPID(0x80,p,i,d,q)
	r.SetM2VelocityPID(0x80,p,i,d,q)
	r.SetM1VelocityPID(0x81,p,i,d,q)

def consistentPIDQ(m):
        PIDQ = readPIDQ()
        vals = PIDQ[m][1:]
        setAllPIDQ(vals[0],vals[1],vals[2],vals[3])

def spin(speed):
	r.SpeedM1(0x80, speed)
	r.SpeedM2(0x80, -speed)
	r.SpeedM1(0x81, speed)

def forward(speed):
	r.SpeedM1(0x80, speed)
	r.SpeedM2(0x80, speed)

def stop():
	r.ForwardM1(0x80, 0)
	r.ForwardM2(0x80, 0)
	r.ForwardM1(0x81, 0)

def getAvgSpeed(addr,m,speed):
	f = r.ForwardM1 if m == 1 else r.ForwardM2
	s = r.ReadSpeedM1 if m == 1 else r.ReadSpeedM2

	# Stop all motors and reset encoder counts
	stop()
	r.ResetEncoders(addr)

	# Call the correct function (M1 or M2)	
	f(addr,speed)

	# Let the motor ramp up and get stable
	time.sleep(1)

	# how many samples should I get?
	#N = 10

	# how many seconds to sample over?
	Tmax = 5

	Ts = 0.100 # 100 ms, sample rate

	# Based on that, how many samples will I be getting?
	N = int(math.floor(Tmax/Ts))

	samples = []
	while len(samples)<N:
		samples.append(s(addr)[1])
		time.sleep(Ts)
	
	avg = sum(samples)/N

	# Stop motors
	stop()

	return (avg,N)
