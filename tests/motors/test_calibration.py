import wheelbase as w
import motion

w.init()

scale = .3
motion.drive(scale,0,0)
time.sleep(1)
motion.drive(0,0,0)
time.sleep(1)
motion.drive(-scale,0,0)
time.sleep(1)
motion.drive(0,0,0)
time.sleep(1)
motion.drive(scale*math.cos(2*math.pi/3),scale*math.sin(2*math.pi/3),0)
time.sleep(1)
motion.drive(0,0,0)
time.sleep(1)
motion.drive(-math.cos(2*math.pi/3)*scale,-math.sin(2*math.pi/3)*scale,0)
time.sleep(1)
motion.drive(0,0,0)
time.sleep(1)
motion.drive(math.cos(-2*math.pi/3)*scale,math.sin(-2*math.pi/3)*scale,0)
time.sleep(1)
motion.drive(0,0,0)
time.sleep(1)
motion.drive(-math.cos(-2*math.pi/3)*scale,-math.sin(-2*math.pi/3)*scale,0)
time.sleep(1)
motion.drive(0,0,0)