import wheelbase as w
import motion
import time
import numpy as np

w.init()

scale = .3
time_between = 1

# Forward
motion.drive(scale,0,0)
time.sleep(time_between)

motion.stop()
time.sleep(time_between)

# Backward
motion.drive(-scale,0,0)
time.sleep(time_between)

motion.stop()
time.sleep(time_between)

# Left back
motion.drive(scale*np.cos(2*np.pi/3),scale*np.sin(2*np.pi/3),0)
time.sleep(time_between)

motion.stop()
time.sleep(time_between)

# Right forward
motion.drive(-np.cos(2*np.pi/3)*scale,-np.sin(2*np.pi/3)*scale,0)
time.sleep(time_between)

motion.stop()
time.sleep(time_between)

# Right back
motion.drive(np.cos(-2*np.pi/3)*scale,np.sin(-2*np.pi/3)*scale,0)
time.sleep(time_between)

motion.stop()
time.sleep(time_between)

# Right Forward
motion.drive(-np.cos(-2*np.pi/3)*scale,-np.sin(-2*np.pi/3)*scale,0)
time.sleep(time_between)

motion.stop()