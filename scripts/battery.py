#!/usr/bin/env python

import rcv3.roboclaw as r

r.Open('/dev/ttySAC0', 38400)

print("Battery: {}v".format(r.ReadMainBatteryVoltage(0x80)[1]/10.0))
