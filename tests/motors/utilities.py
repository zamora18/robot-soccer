import wheelbase as w
import param as p
import time, math

def readPIDQ():
	PIDQ = []
	PIDQ.append(w.ReadVelocityPID(w.M1))
	PIDQ.append(w.ReadVelocityPID(w.M2))
	PIDQ.append(w.ReadVelocityPID(w.M3))
	return PIDQ

def setAllPIDQ(p,i,d,q):
	w.SetVelocityPID(w.M1,p,i,d,q)
	w.SetVelocityPID(w.M2,p,i,d,q)
	w.SetVelocityPID(w.M3,p,i,d,q)

def consistentPIDQ(m):
        PIDQ = readPIDQ()
        vals = PIDQ[m][1:]
        setAllPIDQ(vals[0],vals[1],vals[2],vals[3])

def spin(speed):
	w.Speed(w.M1, speed)
	w.Speed(w.M2, -speed)
#	w.Speed(w.M3, speed)

def forward(speed):
	w.Speed(w.M1, speed)
	w.Speed(w.M2, speed)

def getAvgSpeed(motor_id,speed):

	# Stop all motors and reset encoder counts
	w.kill()
	w.ResetEncoders(motor_id)

	# Move forward
	w.Forward(motor_id, speed)

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
	w.kill()

	return (avg,N)