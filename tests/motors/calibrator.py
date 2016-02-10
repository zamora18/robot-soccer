import time, math

import numpy as np

import wheelbase as w
import motion


MAX_PWM = 127
kp = 3.991973876953125
ki = 1.9959869384765625
kd = 5.969512939453125
q  = 308419

speedM1 = None
speedM2 = None
speedM3 = None

wheelbase_configured = False

def _print_stats():
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

def _read(motor_id):
    samples = 2
    result = 0
    for i in range(0, samples):
        sample = 0
        s,sample,a = w.ReadSpeed(motor_id)
        #print sample
        result = result + sample
    
    result = result/samples
    return result

def calibrate(speed=48,sleep_time=2,set_PID=True):
    speedM1Forward  = 0
    speedM1Backward = 0
    speedM2Forward  = 0
    speedM2Backward = 0
    speedM3Forward  = 0
    speedM3Backward = 0

    # Initialize wheelbase with PID
    w.init(set_PID=False)
    global wheelbase_configured
    wheelbase_configured = True

    w.kill()
    _print_stats()

    # Forward (at 0 degrees)
    w.Backward(w.M1,speed) # M1 backward sample 1
    w.Forward(w.M3,speed)  # M3 forward sample 1
    time.sleep(sleep_time)

    speedM1Backward = speedM1Backward + _read(w.M1)
    speedM3Forward  = speedM3Forward  + _read(w.M3)

    w.kill()
    time.sleep(1)

    # Backward (at 180 degrees)
    w.Forward(w.M1,speed)  # M1 forward sample 1
    w.Backward(w.M3,speed) # M3 backward sample 1
    time.sleep(sleep_time)

    speedM1Forward  = speedM1Forward  + _read(w.M1)
    speedM3Backward = speedM3Backward + _read(w.M3)

    w.kill()
    time.sleep(1)

    # Right back (at -120 degrees)
    w.Backward(w.M3,speed) # M3 backward sample 2 
    w.Forward(w.M2,speed)  # M2 forward sample 1
    time.sleep(sleep_time)

    speedM3Backward = speedM3Backward + _read(w.M3)
    # speedM3Backward = speedM3Backward/2
    speedM2Forward  = speedM2Forward + _read(w.M2)

    w.kill();
    time.sleep(1);

    # Right forward (at 120 degrees)
    w.Forward(w.M3,speed); # M3 forward sample 2
    w.Backward(w.M2,speed); # M2 backward sample 1
    time.sleep(sleep_time)

    speedM3Forward  = speedM3Forward + _read(w.M3)
    # speedM3Forward  = speedM3Forward/2
    speedM2Backward = speedM2Backward + _read(w.M2)

    w.kill();
    time.sleep(1);

    # Left Back (at 120 degrees)
    w.Forward(w.M1,speed);  # M1 forward sample 2
    w.Backward(w.M2,speed); # M2 backward sample 2
    time.sleep(sleep_time)

    speedM1Forward  = speedM1Forward + _read(w.M1)
    # speedM1Forward  = speedM1Forward/2
    speedM2Backward = speedM2Backward + _read(w.M2)
    # speedM2Backward = speedM2Backward/2

    w.kill();
    time.sleep(1);

    # Left Forward (at -120 degrees)
    w.Backward(w.M1,speed); # M1 backward sample 2
    w.Forward(w.M2,speed);  # M2 forward sample 2
    time.sleep(sleep_time)

    speedM1Backward = speedM1Backward + _read(w.M1)
    # speedM1Backward = speedM1Backward/2
    speedM2Forward  = speedM2Forward + _read(w.M2)
    # speedM2Forward  = speedM2Forward/2

    w.kill();

    speedM1Forward  = (speedM1Forward*MAX_PWM)/speed
    speedM1Backward = (speedM1Backward*MAX_PWM)/speed
    speedM2Forward  = (speedM2Forward*MAX_PWM)/speed
    speedM2Backward = (speedM2Backward*MAX_PWM)/speed
    speedM3Forward  = (speedM3Forward*MAX_PWM)/speed
    speedM3Backward = (speedM3Backward*MAX_PWM)/speed

    global speedM1
    global speedM2
    global speedM3

    speedM1 = (speedM1Forward + abs(speedM1Backward))/2
    speedM2 = (speedM2Forward + abs(speedM2Backward))/2
    speedM3 = (speedM3Forward + abs(speedM3Backward))/2

    print "M1QPPS={},M2QPPS={},M3QPPS={}".format(speedM1,speedM2,speedM3)

    if set_PID:
        w.SetVelocityPID(w.M1,kp,ki,kd,speedM1)
        w.SetVelocityPID(w.M2,kp,ki,kd,speedM2)
        w.SetVelocityPID(w.M3,kp,ki,kd,speedM3)

        _print_stats()

def test_calibration(velocity=0.6,sleep_time=1.5,
                    M1QPPS=None,M2QPPS=None,M3QPPS=None):
    global wheelbase_configured

    # Make sure everything is init'd
    if not wheelbase_configured:
        w.init()
        wheelbase_configured = True

    _m1qpps = M1QPPS if M1QPPS is not None else speedM1
    _m2qpps = M2QPPS if M2QPPS is not None else speedM2
    _m3qpps = M3QPPS if M3QPPS is not None else speedM3

    if _m1qpps is not None and _m2qpps is not None and _m3qpps is not None:
        w.UpdateVelocityPID(w.M1, q=_m1qpps)
        w.UpdateVelocityPID(w.M2, q=_m2qpps)
        w.UpdateVelocityPID(w.M3, q=_m3qpps)

    _print_stats()

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

def test_square_calibration(velocity=0.6,sleep_time=1.5,
                    M1QPPS=None,M2QPPS=None,M3QPPS=None):
    global wheelbase_configured

    # Make sure everything is init'd
    if not wheelbase_configured:
        w.init()
        wheelbase_configured = True

    _m1qpps = M1QPPS if M1QPPS is not None else speedM1
    _m2qpps = M2QPPS if M2QPPS is not None else speedM2
    _m3qpps = M3QPPS if M3QPPS is not None else speedM3

    if _m1qpps is not None and _m2qpps is not None and _m3qpps is not None:
        w.SetVelocityPID(w.M1, kp, ki, kd, _m1qpps)
        w.SetVelocityPID(w.M2, kp, ki, kd, _m2qpps)
        w.SetVelocityPID(w.M3, kp, ki, kd, _m3qpps)


    # Right
    motion.drive(velocity,0,0)
    time.sleep(sleep_time)

    motion.stop()
    time.sleep(sleep_time)

    # Up
    motion.drive(0,velocity,0)
    time.sleep(sleep_time)

    motion.stop()
    time.sleep(sleep_time)

    # Left
    motion.drive(-velocity,0,0)
    time.sleep(sleep_time)

    motion.stop()
    time.sleep(sleep_time)

    # Down
    motion.drive(0,-velocity,0)
    time.sleep(sleep_time)

    motion.stop()