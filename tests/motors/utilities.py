import time, math

import numpy as np

import wheelbase as w
import param as p
import motion

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

class MotorCalibrator(object):
    """MotorCalibrator"""
    MAX_PWM = 127
    kp = 3.991973876953125
    ki = 1.9959869384765625
    kd = 5.969512939453125
    q  = 308419

    speedM1 = 0
    speedM2 = 0
    speedM3 = 0

    wheelbase_configured = False

    def __init__(self):
        super(MotorCalibrator, self).__init__()

    def _print_stats(self):
        print w.ReadMainBatteryVoltage()
        s,p1,i1,d1,q1 = w.ReadVelocityPID(w.M1)
        print "M1 P=%.2f" % p1
        print "M1 I=%.2f" % i1
        print "M1 D=%.2f" % d1
        print "M1 QPPS=",q1
        s,p2,i2,d2,q2 = w.ReadVelocityPID(w.M2)
        print "M2 P=%.2f" % p2
        print "M2 I=%.2f" % i2
        print "M2 D=%.2f" % d2
        print "M2 QPPS=",q2
        s,p3,i3,d3,q3 = w.ReadVelocityPID(w.M3)
        print "M3 P=%.2f" % p3
        print "M3 I=%.2f" % i3
        print "M3 D=%.2f" % d3
        print "M3 QPPS=",q3
        print "\n\n"

    def _read(self,motor_id):
        samples = 2
        result = 0
        for i in range(0, samples):
        sample = 0
        s,sample,a = w.ReadSpeed(motor_id)
        #print sample
        result = result + sample
        result = result/samples
        return result

    def calibrate(self,speed=48,sleep_time=2):
        speedM1Forward  = 0
        speedM1Backward = 0
        speedM2Forward  = 0
        speedM2Backward = 0
        speedM3Forward  = 0
        speedM3Backward = 0

        # Initialize wheelbase with PID
        w.init()
        self.wheelbase_configured = True

        w.kill()
        _print_stats()

        # Forward
        w.Backward(w.M1,speed) # M1 backward sample 1
        w.Forward(w.M3,speed)  # M3 forward sample 1
        time.sleep(sleep_time)

        speedM1Backward = speedM1Backward + _read(w.M1)
        speedM3Forward  = speedM3Forward  + _read(w.M3)

        w.kill()
        time.sleep(1)

        # Backward
        w.Forward(w.M1,speed)  # M1 forward sample 1
        w.Backward(w.M3,speed) # M3 backward sample 1
        time.sleep(sleep_time)

        speedM1Forward  = speedM1Forward  + _read(w.M1)
        speedM3Backward = speedM3Backward + _read(w.M3)

        w.kill()
        time.sleep(1)

        # Left back
        w.Backward(w.M3,speed) # M3 backward sample 2 
        w.Forward(w.M2,speed)  # M2 forward sample 1
        time.sleep(sleep_time)

        speedM3Backward = speedM3Backward + _read(w.M3)
        speedM3Backward = speedM3Backward/2
        speedM2Forward  = speedM2Forward + _read(w.M2)

        w.kill();
        time.sleep(1);

        # Left forward
        w.Forward(w.M3,speed); # M3 forward sample 2
        w.Backward(w.M2,speed); # M2 backward sample 1
        time.sleep(sleep_time)

        speedM3Forward  = speedM3Forward + _read(w.M3)
        speedM3Forward  = speedM3Forward/2
        speedM2Backward = speedM2Backward + _read(w.M2)

        w.kill();
        time.sleep(1);

        # RightBack
        w.Forward(w.M1,speed);  # M1 forward sample 2
        w.Backward(w.M2,speed); # M2 backward sample 2
        time.sleep(sleep_time)

        speedM1Forward  = speedM1Forward + _read(w.M1)
        speedM1Forward  = speedM1Forward/2
        speedM2Backward = speedM2Backward + _read(w.M2)
        speedM2Backward = speedM2Backward/2

        w.kill();
        time.sleep(1);

        # Right Forward
        w.Backward(w.M1,speed); # M1 backward sample 2
        w.Forward(w.M2,speed);  # M2 forward sample 2
        time.sleep(sleep_time)

        speedM1Backward = speedM1Backward + _read(w.M1)
        speedM1Backward = speedM1Backward/2
        speedM2Forward  = speedM2Forward + _read(w.M2)
        speedM2Forward  = speedM2Forward/2

        w.kill();

        speedM1Forward  = (speedM1Forward*MAX_PWM)/speed
        speedM1Backward = (speedM1Backward*MAX_PWM)/speed
        speedM2Forward  = (speedM2Forward*MAX_PWM)/speed
        speedM2Backward = (speedM2Backward*MAX_PWM)/speed
        speedM3Forward  = (speedM3Forward*MAX_PWM)/speed
        speedM3Backward = (speedM3Backward*MAX_PWM)/speed

        self.speedM1 = (speedM1Forward - speedM1Backward)/2
        self.speedM2 = (speedM2Forward - speedM2Backward)/2
        self.speedM3 = (speedM3Forward - speedM3Backward)/2

        w.SetVelocityPID(w.M1,self.kp,self.ki,self.kd,self.speedM1)
        w.SetVelocityPID(w.M2,self.kp,self.ki,self.kd,self.speedM2)
        w.SetVelocityPID(w.M3,self.kp,self.ki,self.kd,self.speedM3)

        _print_stats()

    def test_calibration(self,velocity=0.3,sleep_time=1,
                        M1QPPS=None,M2QPPS=None,M3QPPS=None):

        # Make sure everything is init'd
        if not self.wheelbase_configured:
            w.init()
            self.wheelbase_configured = True

        _m1qpps = M1QPPS if M1QPPS is not None else self.speedM1
        _m2qpps = M2QPPS if M2QPPS is not None else self.speedM2
        _m3qpps = M3QPPS if M3QPPS is not None else self.speedM3

        if _m1qpps is not None and _m2qpps is not None and _m3qpps is not None:
            w.SetVelocityPID(w.M1, self.kp, self.ki, self.kd, _m1qpps)
            w.SetVelocityPID(w.M2, self.kp, self.ki, self.kd, _m2qpps)
            w.SetVelocityPID(w.M3, self.kp, self.ki, self.kd, _m3qpps)


        # Forward
        motion.drive(velocity,0,0)
        time.sleep(sleep_time)

        motion.stop()
        time.sleep(sleep_time)

        # Backward
        motion.drive(-velocity,0,0)
        time.sleep(sleep_time)

        motion.stop()
        time.sleep(sleep_time)

        # Left back
        motion.drive(velocity*np.cos(2*np.pi/3),velocity*np.sin(2*np.pi/3),0)
        time.sleep(sleep_time)

        motion.stop()
        time.sleep(sleep_time)

        # Right forward
        motion.drive(-np.cos(2*np.pi/3)*velocity,-np.sin(2*np.pi/3)*velocity,0)
        time.sleep(sleep_time)

        motion.stop()
        time.sleep(sleep_time)

        # Right back
        motion.drive(np.cos(-2*np.pi/3)*velocity,np.sin(-2*np.pi/3)*velocity,0)
        time.sleep(sleep_time)

        motion.stop()
        time.sleep(sleep_time)

        # Right Forward
        motion.drive(-np.cos(-2*np.pi/3)*velocity,-np.sin(-2*np.pi/3)*velocity,0)
        time.sleep(sleep_time)

        motion.stop()