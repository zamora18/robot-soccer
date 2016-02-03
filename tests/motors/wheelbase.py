from rcv3 import roboclaw as r
from functools import partial

MOTOR_COUNT = 3

_SPEED_MAX = 127
_SPEED_MIN = 0

M1 = 0
M2 = 1
M3 = 2

_RC = [
		{ 'addr': 0x80, 'motor': 'M1' },
		{ 'addr': 0x81, 'motor': 'M1' },
		{ 'addr': 0x80, 'motor': 'M2' },
	 ]

def _getFunction(func_str, motor_id):
	"""Get Function
	"""

	# Based on the motor_id, get the correct roboclaw address and motor
	motor_str = _RC[motor_id]['motor']
	addr = _RC[motor_id]['addr']

	# Using the func_str build the correct roboclaw method
	func_str = func_str.format(motor_str)

	# Go get the correct method from the 'r' module
	func = getattr(r, func_str)

	# Return a curried version of func, with the address already applied
	return partial(func, addr)

def _map(x, in_min, in_max, out_min, out_max):
	"""Map
	Takes in a value x and maps it from one range to another
	"""
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def Forward(motor_id, speed):
	func = _getFunction('Forward{}', motor_id)
	#speed = _map(speed, 0, 100, _SPEED_MIN, _SPEED_MAX)
	return func(speed)

def Backward(motor_id, speed):
	func = _getFunction('Backward{}', motor_id)
	#speed = _map(speed, 0, 100, _SPEED_MIN, _SPEED_MAX)
	return func(speed)

def Speed(motor_id, speed):
	func = _getFunction('Speed{}', motor_id)
	return func(speed)

def ReadVelocityPID(motor_id):
	func = _getFunction('Read{}VelocityPID', motor_id)
	return func()

def SetVelocityPID(motor_id, p, i, d, q):
	func = _getFunction('Set{}VelocityPID', motor_id)
	return func(p, i, d, q)

def ResetEncoders(motor_id):
	func = _getFunction('ResetEncoders', motor_id)
	return func()

def Kill():
	for motor_id in range(MOTOR_COUNT):
		Forward(motor_id, 0)
		Speed(motor_id, 0)


def init():
	r.Open('/dev/ttySAC0', 38400)

	# PID stuff here?
	SetVelocityPID(0, 3.991973876953125, 1.9959869384765625, 5.969512939453125, 308420)
	SetVelocityPID(1, 3.991973876953125, 1.9959869384765625, 5.969512939453125, 308420)
	SetVelocityPID(2, 3.991973876953125, 1.9959869384765625, 5.969512939453125, 308420)
