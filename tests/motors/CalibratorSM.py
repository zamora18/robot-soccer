import time, math, random

import wheelbase as w

_state = 'START'

_counter = 0

timer_rate_ms = 100

_locked = True

DRIVE_TIME = (2000/timer_rate_ms)
_SLEEP_TIME = (1000/timer_rate_ms)

_next_state = ''

DEFAULT_SPEED_M1 = 48
DEFAULT_SPEED_M2 = 48
DEFAULT_SPEED_M3 = 48

def _stop():
    w.Forward(w.M1,0)
    w.Forward(w.M2,0)
    w.Forward(w.M3,0)

def _change_state(new_state, after=None, next=None):
    global _state
    global _next_state
    global _counter

    if after:
        # Only change state if 'after' has elapsed
        if _counter < after:
            return

    _counter = 0
    _state = new_state

    if next:
        _next_state = next

def _print_stats():
    try:
        print w.ReadMainBatteryVoltage()
        s,p1,i1,d1,q1 = w.ReadVelocityPID(w.M1)
        s,p2,i2,d2,q2 = w.ReadVelocityPID(w.M2)
        s,p3,i3,d3,q3 = w.ReadVelocityPID(w.M3)
    except:
        p1 = i1 = d1 = q1 = 0
        p2 = i2 = d2 = q2 = 0
        p3 = i3 = d3 = q3 = 0
        
        print "M1 P=%.2f" % p1
        print "M1 I=%.2f" % i1
        print "M1 D=%.2f" % d1
        print "M1 QPPS=",q1
        
        print "M2 P=%.2f" % p2
        print "M2 I=%.2f" % i2
        print "M2 D=%.2f" % d2
        print "M2 QPPS=",q2

        print "M3 P=%.2f" % p3
        print "M3 I=%.2f" % i3
        print "M3 D=%.2f" % d3
        print "M3 QPPS=",q3
        print "\n\n"


_motors = ('M1', 'M2', 'M3')

_speeds = {
    'M1': DEFAULT_SPEED_M1,
    'M2': DEFAULT_SPEED_M2,
    'M3': DEFAULT_SPEED_M3
}

_samples = {
    'M1': { 'forward': [], 'backward': [] },
    'M2': { 'forward': [], 'backward': [] },
    'M3': { 'forward': [], 'backward': [] },
}

def _add_sample(motor_id, sample, sample_type):
    global _samples

    m = _motors[motor_id]
    s = _speeds[m]
    _samples[m][sample_type].append({ 'speed': s, 'sample': sample })

def _sample(forward=None, backward=None):
    # Only start sampling halfway into the drive
    if _counter > DRIVE_TIME/2:

        # Grab a sample from forward motor
        try:
            s,sample,a = w.ReadSpeed(forward)
        except:
            sample = random.randint(100,1000) # for testing

        _add_sample(forward, sample, 'forward')

        # Grab a sample from backward motor
        try:
            s,sample,a = w.ReadSpeed(backward)
        except:
            sample = random.randint(1000,10000) # for testing

        _add_sample(backward, sample, 'backward')

def _get_speed(motor_id):
    m = _motors[motor_id]
    return _speeds[m]

def init():
    w.init(set_PID=False)

def start():
    global _locked
    _locked = False

def stop():
    global _locked
    _locked = True

    _stop()
    _state = 'START'

def set_m1_speed(val):
    m = _motors[w.M1]
    _speeds[m] = val

def set_m2_speed(val):
    m = _motors[w.M2]
    _speeds[m] = val

def set_m3_speed(val):
    m = _motors[w.M3]
    _speeds[m] = val

def tick():
    global _counter

    print (_state,_counter)

    # Only run the SM if it's unlocked
    if _locked:
        return

    _counter = _counter + 1

    if _state == 'START':
        _print_stats()

        _change_state('FORWARD')

    # elif _state == 'STOP':
    #     _stop()

        # 

    # elif _state == 'SLEEP_BUT_DRIVE':
    #     if _counter == _SLEEP_TIME_DRIVE:
    #         _change_state(_next_state)

    elif _state == 'SLEEP':
        _stop()
        _change_state(_next_state, after=_SLEEP_TIME)



    elif _state == 'FORWARD':
        w.Backward(w.M1,_get_speed(w.M1))
        w.Forward(w.M3,_get_speed(w.M3))

        _sample(backward=w.M1, forward=w.M3)

        _change_state('SLEEP', after=DRIVE_TIME, next='BACKWARD')

    elif _state == 'BACKWARD':
        w.Forward(w.M1,_get_speed(w.M1))
        w.Backward(w.M3,_get_speed(w.M3))

        _sample(forward=w.M1, backward=w.M3)

        _change_state('SLEEP', after=DRIVE_TIME, next='RIGHT_BACK')



    elif _state == 'RIGHT_BACK':
        w.Backward(w.M3,_get_speed(w.M3))
        w.Forward(w.M2,_get_speed(w.M2))

        _sample(backward=w.M3, forward=w.M2)

        _change_state('SLEEP', after=DRIVE_TIME, next='RIGHT_FORWARD')

    elif _state == 'RIGHT_FORWARD':
        w.Forward(w.M3,_get_speed(w.M3))
        w.Backward(w.M2,_get_speed(w.M2))

        _sample(forward=w.M3, backward=w.M2)

        _change_state('SLEEP', after=DRIVE_TIME, next='LEFT_BACK')



    elif _state == 'LEFT_BACK':
        w.Forward(w.M1,_get_speed(w.M1))
        w.Backward(w.M2,_get_speed(w.M2))

        _sample(forward=w.M1, backward=w.M2)

        _change_state('SLEEP', after=DRIVE_TIME, next='LEFT_FORWARD')

    elif _state == 'LEFT_FORWARD':
        w.Backward(w.M1,_get_speed(w.M1))
        w.Forward(w.M2,_get_speed(w.M2))

        _sample(backward=w.M1, forward=w.M2)

        _change_state('SLEEP', after=DRIVE_TIME, next='CALCULATE')


    elif _state == 'CALCULATE':
        pass