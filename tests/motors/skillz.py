import motion
import time

def main():
	pass

def spin():
	motion.drive(0, 0, 1)

	time.sleep(5)

	motion.stop()

def square():
	motion.drive(1, 0, 0)
	time.sleep(1)

	motion.drive(0, 1, 0)
	time.sleep(1)

	motion.drive(-1, 0, 0)
	time.sleep(1)

	motion.drive(0, -1, 0)
	time.sleep(1)

	motion.stop()