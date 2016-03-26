#!/usr/bin/env python

import os

os.system("echo 1 > /sys/class/gpio/gpio200/value; sleep .07; echo 0 > /sys/class/gpio/gpio200/value")
