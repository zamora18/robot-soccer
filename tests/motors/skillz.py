import motion
import time
import numpy as np

def main():
	pass

def spin():
	motion.drive(0, 0, 2*np.pi)

	time.sleep(3)

	motion.stop()

def square():
	motion.drive(.5, 0, 0)
	time.sleep(1)

	motion.stop()

	motion.drive(0, .5, 0)
	time.sleep(1)

	motion.stop()

	motion.drive(-.5, 0, 0)
	time.sleep(1)

	motion.stop()

	motion.drive(0, -.5, 0)
	time.sleep(1)

	motion.stop()