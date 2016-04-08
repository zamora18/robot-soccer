#!/usr/bin/env python

# Choose the right roboclaw library
r = None
if os.environ['USE_RCV3'] == 'false':
    r = importlib.import_module('rcv3.roboclaw')
else:
    r = importlib.import_module('rcv5.roboclaw')

r.Open('/dev/ttySAC0', 38400)

print("Battery: {}v".format(r.ReadMainBatteryVoltage(0x80)[1]/10.0))
