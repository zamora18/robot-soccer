#!/usr/bin/env python
import sys, os
import importlib

# Choose the right roboclaw library
r = None
if os.environ['USE_RCV3'] == 'false':
    r = importlib.import_module('rcv3.roboclaw')
else:
    r = importlib.import_module('rcv5.roboclaw')

r.Open('/dev/ttySAC0', 38400)

r.SpeedM1(0x80, 0)
r.SpeedM2(0x80, 0)
r.SpeedM1(0x81, 0)
r.SpeedM2(0x81, 0)

r.ForwardM1(0x80, 0)
r.ForwardM2(0x80, 0)
r.ForwardM1(0x81, 0)
r.ForwardM2(0x81, 0)
