import wheelbase as w
import time


kp = 3.991973876953125
ki = 1.9959869384765625
kd = 5.969512939453125
q  = 308419

def print_stats():
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

def read(motor_id):
  samples = 2
  result = 0
  for i in range(0, samples):
    sample = 0
    s,sample,a = w.ReadSpeed(motor_id)
    #print sample
    result = result + sample
  result = result/samples
  return result
  
speedM1Forward=0
speedM1Backward=0
speedM2Forward=0
speedM2Backward=0
speedM3Forward=0
speedM3Backward=0

speed = 48
timeBetweenDrives = 2

w.init()

w.kill()
print_stats()

# Krum 1 == Our 1
# Krum 2 == Our 3
# Krum 3 == Our 2

# Forward
w.Backward(w.M1,speed) # M1 backward sample 1
w.Forward(w.M3,speed)  # M3 forward sample 1
time.sleep(timeBetweenDrives)

speedM1Backward = speedM1Backward + read(w.M1)
speedM3Forward  = speedM3Forward  + read(w.M3)

w.kill()
time.sleep(1)

# Backward
w.Forward(w.M1,speed)  # M1 forward sample 1
w.Backward(w.M3,speed) # M3 backward sample 1
time.sleep(timeBetweenDrives)

speedM1Forward  = speedM1Forward  + read(w.M1)
speedM3Backward = speedM3Backward + read(w.M3)

w.kill()
time.sleep(1)

# Left back
w.Backward(w.M3,speed) # M3 backward sample 2 
w.Forward(w.M2,speed)  # M2 forward sample 1
time.sleep(timeBetweenDrives)

speedM3Backward = speedM3Backward + read(w.M3)
speedM3Backward = speedM3Backward/2
speedM2Forward  = speedM2Forward + read(w.M2)

w.kill();
time.sleep(1);

# Left forward
w.Forward(w.M3,speed); # M3 forward sample 2
w.Backward(w.M2,speed); # M2 backward sample 1
time.sleep(timeBetweenDrives)

speedM3Forward  = speedM3Forward + read(w.M3)
speedM3Forward  = speedM3Forward/2
speedM2Backward = speedM2Backward + read(w.M2)

w.kill();
time.sleep(1);

# RightBack
w.Forward(w.M1,speed);  # M1 forward sample 2
w.Backward(w.M2,speed); # M2 backward sample 2
time.sleep(timeBetweenDrives)

speedM1Forward  = speedM1Forward + read(w.M1)
speedM1Forward  = speedM1Forward/2
speedM2Backward = speedM2Backward + read(w.M2)
speedM2Backward = speedM2Backward/2

w.kill();
time.sleep(1);

# Right Forward
w.Backward(w.M1,speed); # M1 backward sample 2
w.Forward(w.M2,speed);  # M2 forward sample 2
time.sleep(timeBetweenDrives)

speedM1Backward = speedM1Backward + read(w.M1)
speedM1Backward = speedM1Backward/2
speedM2Forward  = speedM2Forward + read(w.M2)
speedM2Forward  = speedM2Forward/2

w.kill();

speedM1Forward  = (speedM1Forward*127)/speed
speedM1Backward = (speedM1Backward*127)/speed
speedM2Forward  = (speedM2Forward*127)/speed
speedM2Backward = (speedM2Backward*127)/speed
speedM3Forward  = (speedM3Forward*127)/speed
speedM3Backward = (speedM3Backward*127)/speed

print "All speeds:"
print "\t{}".format(speedM1Forward)
print "\t{}".format(speedM1Backward)
print "\t{}".format(speedM2Forward)
print "\t{}".format(speedM2Backward)
print "\t{}".format(speedM3Forward)
print "\t{}".format(speedM3Backward)

speedM1 = (speedM1Forward - speedM1Backward)/2
speedM2 = (speedM2Forward - speedM2Backward)/2
speedM3 = (speedM3Forward - speedM3Backward)/2

w.SetVelocityPID(w.M1,kp,ki,kd,speedM1)
w.SetVelocityPID(w.M2,kp,ki,kd,speedM2)
w.SetVelocityPID(w.M3,kp,ki,kd,speedM3)

print_stats()
