#!/usr/bin/env python

import rcv3.roboclaw as r

r.Open('/dev/ttySAC0', 38400)

r.SpeedM1(0x80, 0)
r.SpeedM2(0x80, 0)
r.SpeedM1(0x81, 0)
r.SpeedM2(0x81, 0)

r.ForwardM1(0x80, 0)
r.ForwardM2(0x80, 0)
r.ForwardM1(0x81, 0)
r.ForwardM2(0x81, 0)
