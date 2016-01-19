import serial
import struct
import time

class RoboClaw:

	def __init__(self, addr, port, baudrate):
		self.roboserial = RoboSerial(addr, port, baudrate)
		self.addr = addr

	def _checksums_match(self):
		recvd_checksum = self.roboserial.get_recvd_checksum()
		checksum = self.roboserial.create_checksum()
		return (recvd_checksum == checksum)

	def drive_forward_m1(self,speed):
		"""Drive motor 1 forward.

		Valid data range is 0-127. 0 = full stop. 127 = full speed forward.
		"""
		self.roboserial.send_command(Cmd.M1FORWARD, speed)

	def drive_backward_m1(self,speed):
		"""Drive motor 1 backward.

		Valid data range is 0-127. 0 = full stop. 127 = full speed backward.
		"""
		self.roboserial.send_command(Cmd.M1BACKWARD, speed)

	def set_min_main_voltage(self,value):
		"""Sets main battery (B- / B+) minimum voltage level.

		If the battery voltage drops below the set voltage level RoboClaw
		will shut down. The value is cleared at start up and must be set
		after each power up. The voltage is set in .2 volt increments. A
		value of 0 sets minimum voltage allowed, which is 6V.

		Valid data range is 0-120 (6V-30V).

		value = (Desired Volts - 6) x 5
		"""
		self.roboserial.send_command(Cmd.SETMINMB, value)

	def set_max_main_voltage(self,value):
		"""Sets main battery (B- / B+) maximum voltage level.

		If you are using a battery of any type you can ignore this setting.
		During regenerative breaking a back voltage is applied to charge
		the battery. When using an ATX type power supply if it senses
		anything over 16V it will shut down. By setting the maximum voltage
		level, RoboClaw before exceeding it will go into hard breaking mode
		until the voltage drops below the maximum value set.

		Valid data range is 0-154 (0V-30V).

		value = Desired Volts x 5.12
		"""
		self.roboserial.send_command(Cmd.SETMAXMB,value)

	def drive_forward_m2(self,speed):
		self.roboserial.send_command(Cmd.M2FORWARD, speed)

	def drive_backward_m2(self,speed):
		self.roboserial.send_command(Cmd.M2BACKWARD, speed)

	def drive_m1(self,value):
		self.roboserial.send_command(Cmd.M17BIT, value)

	def drive_m2(self,value):
		self.roboserial.send_command(Cmd.M27BIT, value)

	###################################
	# Mix Mode Commands
	###################################
	# The following commands are mix mode commands and used to control speed
	# and turn. Before a command is executed valid drive and turn data is 
	# required. You only need to send both data packets once. After receiving
	# both valid drive and turn data RoboClaw will begin to operate. At this
	# point you only need to update turn or drive data.
	###################################

	def mixed_drive_forward(self,speed):
		self.roboserial.send_command(Cmd.MIXEDFORWARD, speed)

	def mixed_drive_backwards(self,speed):
		self.roboserial.send_command(Cmd.MIXEDBACKWARD, speed)

	def mixed_turn_right(self,speed):
		self.roboserial.send_command(Cmd.MIXEDRIGHT, speed)

	def mixed_turn_left(self,speed):
		self.roboserial.send_command(Cmd.MIXEDLEFT, speed)

	def mixed_drive(self,value):
		self.roboserial.send_command(Cmd.MIXEDFB, value)

	def mixed_turn(self,value):
		self.roboserial.send_command(Cmd.MIXEDLR, value)

	###################################
	# Advanced Packet Serial Commands
	###################################

	def read_firmware_version(self):
		self.roboserial.send_command(Cmd.GETVERSION)
		return self.roboserial.read(size=32)

	def read_main_bat_voltage(self):
		pass

	def read_logic_bat_voltage(self):
		pass

	def set_min_logic_voltage_level(self,voltage):
		pass

	def set_max_logic_voltage_level(self,voltage):
		pass

	def read_motor_currents(self):
		self.roboserial.send_command(Cmd.GETCURRENTS)

		# Receive Payload
		M1Cur = self.roboserial.read_word()
		M2Cur = self.roboserial.read_word()

		# Checksum
		if not self._checksums_match():
			return (-1, -1)

		return (M1Cur,M2Cur)

	def read_m1_velocity_PID_QPPS(self):
		return self._read_velocity_PID_QPPS(M1=True)

	def read_m2_velocity_PID_QPPS(self):
		return self._read_velocity_PID_QPPS(M2=True)

	def _read_velocity_PID_QPPS(self,M1=False,M2=False):
		cmd = Cmd.READM2PID if M2 else Cmd.READM1PID
		self.roboserial.send_command(cmd)

		# Receive Payload
		kp = self.roboserial.read_long(True)
		ki = self.roboserial.read_long(True)
		kd = self.roboserial.read_long(True)
		QPPS = self.roboserial.read_long()

		# Checksum
		if not self._checksums_match():
			return (-1, -1, -1, -1)

		return (kp,ki,kd,QPPS)

	def set_main_bat_voltage(self):
		pass

	def set_logic_bat_voltage(self):
		pass

	def read_main_bat_settings(self):
		pass

	def read_logic_bat_settings(self):
		pass

	def read_m1_pos_PID(self):
		return self._read_pos_PID(M1=True)

	def read_m2_pos_PID(self):
		return self._read_pos_PID(M2=True)

	def _read_pos_PID(self,M1=False,M2=False):
		cmd = Cmd.READM2POSPID if M2 else Cmd.READM1POSPID
		self.roboserial.send_command(cmd)

		# Receive Payload
		kp = self.roboserial.read_long(True)
		ki = self.roboserial.read_long(True)
		kd = self.roboserial.read_long(True)
		maxi = self.roboserial.read_long(True)
		deadzone = self.roboserial.read_long(True)
		minpos = self.roboserial.read_long(True)
		maxpos = self.roboserial.read_long(True)

		# Checksum
		if not self._checksums_match():
			return (-1, -1, -1, -1, -1, -1, -1)

		return (kp,ki,kd,maxi,deadzone,minpos,maxpos)

	def read_temp(self):
		self.roboserial.send_command(Cmd.GETTEMP)

		# Receive Payload
		temp = self.roboserial.read_word(True)

		# Checksum
		if not self._checksums_match():
			return -1

		return temp

	def read_error_status(self):
		pass

	def read_encoder_mode(self):
		pass

	def set_m1_encoder_mode(self):
		pass

	def set_m2_encoder_mode(self):
		pass

	def write_to_EEPROM(self):
		pass


	###################################
	# Quadrature Decoding
	###################################

	def read_m1_encoder(self):
		return self._read_encoder(M1=True)

	def read_m2_encoder(self):
		return self._read_encoder(M2=True)

	def _read_encoder(self,M1=False,M2=False):
		"""Read encoder value

		Each pulse from the quadrature encoder will increment or decrement
		the counter depending on the direction of motion.

		status:
		 Bit0: Counter underflow
		 Bit1: 0-forward, 1=backward
		 Bit2: Counter overflow
		"""
		cmd = Cmd.GETM2ENC if M2 else Cmd.GETM1ENC
		self.roboserial.send_command(cmd)

		# receive payload
		count = self.roboserial.read_signed_long()
		status = self.roboserial.read_byte()

		# Since only the lower 3 bits matter
		status = status&0x03

		# Checksum
		if not self._checksums_match():
			return (-1, -1)

		return (count, status)

	def read_m1_speed(self):
		return self._read_speed(M1=True)

	def read_m2_speed(self):
		return self._read_speed(M2=True)

	def _read_speed(self,M1=False,M2=False):
		"""Read counter speed.

		Returned value is in pulses per second. RoboClaw keeps track of
		how many pulses received per second for both decoder channels.

		status: 0-forward, 1-backward
		"""
		cmd = Cmd.GETM2SPEED if M2 else Cmd.GETM1SPEED
		self.roboserial.send_command(cmd)

		# receive payload
		speed = self.roboserial.read_signed_long()
		status = self.roboserial.read_byte()

		# Checksum
		if not self._checksums_match():
			return (-1, -1)

		return (speed, status)

	def reset_encoder_counts(self):
		self.roboserial.send_command(Cmd.RESETENC, force_checksum=True)

	###################################
	# Advanced Motor Control
	###################################

	# def SetM1pidq(addr,p,i,d,qpps):
	# 	sendcommand(addr,28)
	# 	writelong(d)
	# 	writelong(p)
	# 	writelong(i)
	# 	writelong(qpps)
	# 	writebyte(checksum&0x7F)

	# def SetM2pidq(addr,p,i,d,qpps):
	# 	sendcommand(addr,29)
	# 	writelong(d)
	# 	writelong(p)
	# 	writelong(i)
	# 	writelong(qpps)
	# 	writebyte(checksum&0x7F)

	# def readM1instspeed(addr):
	# 	sendcommand(addr,30)
	# 	enc = readslong()
	# 	status = readbyte()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (enc,status)
	# 	return (-1,-1)

	# def readM2instspeed(addr):
	# 	sendcommand(addr,31)
	# 	enc = readslong()
	# 	status = readbyte()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (enc,status)
	# 	return (-1,-1)

	# def SetM1Duty(addr,val):
	# 	sendcommand(addr,32)
	# 	writesword(val)
	# 	writebyte(checksum&0x7F)

	# def SetM2Duty(addr,val):
	# 	sendcommand(addr,33)
	# 	writesword(val)
	# 	writebyte(checksum&0x7F)

	# def SetMixedDuty(addr,m1,m2):
	# 	sendcommand(addr,34)
	# 	writesword(m1)
	# 	writesword(m2)
	# 	writebyte(checksum&0x7F)

	# def SetM1Speed(addr,val):
	# 	sendcommand(addr,35)
	# 	writeslong(val)
	# 	writebyte(checksum&0x7F)

	# def SetM2Speed(addr,val):
	# 	sendcommand(addr,36)
	# 	writeslong(val)
	# 	writebyte(checksum&0x7F)

	# def SetMixedSpeed(addr,m1,m2):
	# 	sendcommand(addr,37)
	# 	writeslong(m1)
	# 	writeslong(m2)
	# 	writebyte(checksum&0x7F)

	# def SetM1SpeedAccel(addr,accel,speed):
	# 	sendcommand(addr,38)
	# 	writelong(accel)
	# 	writeslong(speed)
	# 	writebyte(checksum&0x7F)

	# def SetM2SpeedAccel(addr,accel,speed):
	# 	sendcommand(addr,39)
	# 	writelong(accel)
	# 	writeslong(speed)
	# 	writebyte(checksum&0x7F)

	# def SetMixedSpeedAccel(addr,accel,speed1,speed2):
	# 	sendcommand(addr,40)
	# 	writelong(accel)
	# 	writeslong(speed1)
	# 	writeslong(speed2)
	# 	writebyte(checksum&0x7F)

	# def SetM1SpeedDistance(addr,speed,distance,buffer):
	# 	sendcommand(addr,41)
	# 	writeslong(speed)
	# 	writelong(distance)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def SetM2SpeedDistance(addr,speed,distance,buffer):
	# 	sendcommand(addr,42)
	# 	writeslong(speed)
	# 	writelong(distance)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def SetMixedSpeedDistance(addr,speed1,distance1,speed2,distance2,buffer):
	# 	sendcommand(addr,43)
	# 	writeslong(speed1)
	# 	writelong(distance1)
	# 	writeslong(speed2)
	# 	writelong(distance2)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def SetM1SpeedAccelDistance(addr,accel,speed,distance,buffer):
	# 	sendcommand(addr,44)
	# 	writelong(accel)
	# 	writeslong(speed)
	# 	writelong(distance)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def SetM2SpeedAccelDistance(addr,accel,speed,distance,buffer):
	# 	sendcommand(addr,45)
	# 	writelong(accel)
	# 	writeslong(speed)
	# 	writelong(distance)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def SetMixedSpeedAccelDistance(addr,accel,speed1,distance1,speed2,distance2,buffer):
	# 	sendcommand(addr,46)
	# 	writelong(accel)
	# 	writeslong(speed1)
	# 	writelong(distance1)
	# 	writeslong(speed2)
	# 	writelong(distance2)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def readbuffercnts():
	# 	sendcommand(128,47)
	# 	buffer1 = readbyte()
	# 	buffer2 = readbyte()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (buffer1,buffer2)
	# 	return (-1,-1)

	# def readcurrents():
	# 	sendcommand(128,49)
	# 	motor1 = readword()
	# 	motor2 = readword()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (motor1,motor2)
	# 	return (-1,-1)

	# def SetMixedSpeedIAccel(accel1,speed1,accel2,speed2):
	# 	sendcommand(128,50)
	# 	writelong(accel1)
	# 	writeslong(speed1)
	# 	writelong(accel2)
	# 	writeslong(speed2)
	# 	writebyte(checksum&0x7F)

	# def SetMixedSpeedIAccelDistance(accel1,speed1,distance1,accel2,speed2,distance2,buffer):
	# 	sendcommand(128,51)
	# 	writelong(accel1)
	# 	writeslong(speed1)
	# 	writelong(distance1)
	# 	writelong(accel2)
	# 	writeslong(speed2)
	# 	writelong(distance2)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def SetM1DutyAccel(accel,duty):
	# 	sendcommand(128,52)
	# 	writesword(duty)
	# 	writeword(accel)
	# 	writebyte(checksum&0x7F)

	# def SetM2DutyAccel(accel,duty):
	# 	sendcommand(128,53)
	# 	writesword(duty)
	# 	writeword(accel)
	# 	writebyte(checksum&0x7F)

	# def SetMixedDutyAccel(accel1,duty1,accel2,duty2):
	# 	sendcommand(128,54)
	# 	writesword(duty1)
	# 	writeword(accel1)
	# 	writesword(duty2)
	# 	writeword(accel2)
	# 	writebyte(checksum&0x7F)

	# def readM2pidq(addr):
	# 	sendcommand(addr,56)
	# 	p = readlong()
	# 	i = readlong()
	# 	d = readlong()
	# 	qpps = readlong()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (p,i,d,qpps)
	# 	return (-1,-1,-1,-1)

	# def readmainbatterysettings():
	# 	sendcommand(128,59)
	# 	min = readword()
	# 	max = readword()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (min,max)
	# 	return (-1,-1)

	# def readlogicbatterysettings():
	# 	sendcommand(128,60)
	# 	min = readword()
	# 	max = readword()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (min,max)
	# 	return (-1,-1)

	# def SetM1PositionConstants(kp,ki,kd,kimax,deadzone,min,max):
	# 	sendcommand(128,61)
	# 	writelong(kd)
	# 	writelong(kp)
	# 	writelong(ki)
	# 	writelong(kimax)
	# 	writelong(deadzone)
	# 	writelong(min)
	# 	writelong(max)
	# 	writebyte(checksum&0x7F)

	# def SetM2PositionConstants(kp,ki,kd,kimax,deadzone,min,max):
	# 	sendcommand(128,62)
	# 	writelong(kd)
	# 	writelong(kp)
	# 	writelong(ki)
	# 	writelong(kimax)
	# 	writelong(deadzone)
	# 	writelong(min)
	# 	writelong(max)
	# 	writebyte(checksum&0x7F)

	# def readM1PositionConstants():
	# 	sendcommand(128,63)
	# 	p = readlong()
	# 	i = readlong()
	# 	d = readlong()
	# 	imax = readlong()
	# 	deadzone = readlong()
	# 	min = readlong()
	# 	max = readlong()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (p,i,d,imax,deadzone,min,max)
	# 	return (-1,-1,-1,-1,-1,-1,-1)

	# def readM2PositionConstants():
	# 	sendcommand(128,64)
	# 	p = readlong()
	# 	i = readlong()
	# 	d = readlong()
	# 	imax = readlong()
	# 	deadzone = readlong()
	# 	min = readlong()
	# 	max = readlong()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return (p,i,d,imax,deadzone,min,max)
	# 	return (-1,-1,-1,-1,-1,-1,-1)

	# def SetM1SpeedAccelDeccelPosition(accel,speed,deccel,position,buffer):
	# 	sendcommand(128,65)
	# 	writelong(accel)
	# 	writelong(speed)
	# 	writelong(deccel)
	# 	writelong(position)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def SetM2SpeedAccelDeccelPosition(accel,speed,deccel,position,buffer):
	# 	sendcommand(128,66)
	# 	writelong(accel)
	# 	writelong(speed)
	# 	writelong(deccel)
	# 	writelong(position)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def SetMixedSpeedAccelDeccelPosition(accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer):
	# 	sendcommand(128,67)
	# 	writelong(accel1)
	# 	writelong(speed1)
	# 	writelong(deccel1)
	# 	writelong(position1)
	# 	writelong(accel2)
	# 	writelong(speed2)
	# 	writelong(deccel2)
	# 	writelong(position2)
	# 	writebyte(buffer)
	# 	writebyte(checksum&0x7F)

	# def readtemperature():
	# 	sendcommand(128,82)
	# 	val = readword()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return val
	# 	return -1

	# def readerrorstate():
	# 	sendcommand(129,90)
	# 	val = readbyte()
	# 	crc = checksum&0x7F
	# 	if crc==readbyte()&0x7F:
	# 		return val
	# 	return -1
		
	# def readEncoderMode(addr):
	#   sendcommand(addr,91)
	#   mode1 = readbyte()
	#   mode2 = readbyte()
	#   crc = checksum&0x7F
	#   if crc==readbyte()&0x7F:
	#     return (mode1,mode2)
	#   return (-1,-1)
	  
	# def writeSettingsToMem(addr):
	#   sendcommand(addr,94)
	#   crc = checksum&0x7F
	#   if crc==readbyte()&0x7F:
	#     return 0
	#   return -1

	# def calibrateRoboclaws():
	#     p = int(65536 * 4)
	#     i = int(65536 * 2)
	#     d = int(65536 * 6)
	#     #last good calibration readings
	#     voltage = 16.9 # 16.7   # 15.7   # 15.9   # 15.9   # 15.8   # 16.5   # 16.5   # 15.9   # 15.9   # 15.5   # 15.3   # 16.6   # 15.5
	#     qqps_m1 = 142977 # 141606 # 118234 # 129122 # 136502 # 140181 # 146772 # 130185 # 146330 # 149353 # 137669 # 141136 # 148132 # 149287
	#     qqps_m2 = 178091 # 187808 # 139632 # 159086 # 164265 # 164244 # 177244 # 180669 # 180616 # 166407 # 172434 # 165175 # 168984 # 169069
	#     qqps_m3 = 195319 # 175863 # 130377 # 154211 # 171489 # 165285 # 183906 # 181536 # 175021 # 170281 # 159700 # 161999 # 165146 # 164071

	#     read_v = readmainbattery() / 10.0
	    
	#     scale = lambda x: int(x*voltage/read_v)
	#     speedM1 = scale(qqps_m1)
	#     speedM2 = scale(qqps_m2)
	#     speedM3 = scale(qqps_m3)
	    
	#     SetM1pidq(128,p,i,d,speedM1)
	#     SetM2pidq(128,p,i,d,speedM2)
	#     SetM1pidq(129,p,i,d,speedM3)

#print "Roboclaw Example 1\r\n"

#Rasberry Pi/Linux Serial instance example
# port = serial.Serial("/dev/ttySAC0", baudrate=2400, timeout=1.0)

# #Windows Serial instance example
# #port = serial.Serial("COM126", baudrate=38400, timeout=1)
# M1Forward(129,64)
# #M1Forward(0)
# #Get version string
# sendcommand(129,21)
# rcv = port.read(32)
# print repr(rcv)

#sendcommand(129,90)
#rcv = port.read(1)
#print repr(rcv)

#cnt = 0
#while True:
#	cnt=cnt+1
#	print "Count = ",cnt
	
#	print "Error State:",repr(readerrorstate())

#	print "Temperature:",readtemperature()/10.0

#	print "Main Battery:",readmainbattery()/10.0
	
#	print "Logic Battery:",readlogicbattery()/10.0

#	m1cur, m2cur = readcurrents()
#	print "Current M1: ",m1cur/100.0," M2: ",m2cur/100.0
#	
#	min, max = readlogicbatterysettings()
#	print "Logic Battery Min:",min/10.0," Max:",max/10.0
#
#	min, max = readmainbatterysettings()
#	print "Main Battery Min:",min/10.0," Max:",max/10.0
#
#	p,i,d,qpps = readM1pidq()
#	print "M1 P=%.2f" % (p/65536.0)
#	print "M1 I=%.2f" % (i/65536.0)
#	print "M1 D=%.2f" % (d/65536.0)
#	print "M1 QPPS=",qpps
#
#	p,i,d,qpps = readM2pidq()
#	print "M2 P=%.2f" % (p/65536.0)
#	print "M2 I=%.2f" % (i/65536.0)
#	print "M2 D=%.2f" % (d/65536.0)
#	print "M2 QPPS=",qpps
#
#	SetM1DutyAccel(1500,1500)
#	SetM2DutyAccel(1500,-1500)
#	M1Forward(127)
#	M2Backward(127)
#	time.sleep(2)
#	SetM1DutyAccel(1500,-1500)
#	SetM2DutyAccel(1500,1500)
#	M1Forward(0)
#	M2Backward(0)
#	time.sleep(10)
#	M1Backward(127)
#	M2Forward(127)
#	time.sleep(2)

class RoboSerial:

	def __init__(self, addr, port, baudrate):
		self.checksum = 0
		self.addr = addr
		self.port = None;
		self.comm_port = port;
		self.baudrate = baudrate;
		self._open();

	def _open(self):
		self.port = serial.Serial(
			self.comm_port, baudrate=self.baudrate, timeout=1.0)

	def read(self,size=1):
		return self.port.read(size)

	def send_command(self,command,val=None,force_checksum=False):
		value = 0 if val == None else val
		
		# Create the checksum
		self.checksum = (self.addr+command+value)

		# Send the address
		self.port.write(chr(self.addr))

		# Send the command number (p28)
		self.port.write(chr(command))

		if val is not None:
			# Send the value (if there is one)
			self.port.write(chr(value))

		if val is not None or force_checksum:
			# And finally, the checksum!
			self.port.write(chr(self.checksum&0x7F))

	def read_byte(self):
		val = struct.unpack('>B',self.port.read(1))
		self.checksum += val[0]
		return val[0]

	def read_signed_byte(self):
		val = struct.unpack('>b',self.port.read(1))
		self.checksum += val[0]
		return val[0]

	def read_word(self,unscale=False):
		val = struct.unpack('>H',self.port.read(2))
		self.checksum += (val[0]&0xFF)
		self.checksum += (val[0]>>8)&0xFF

		if unscale:
			return val[0]/256.0 # I don't think this is right

		return val[0]

	def read_signed_word(self):
		val = struct.unpack('>h',self.port.read(2))
		self.checksum += val[0]
		self.checksum += (val[0]>>8)&0xFF
		return val[0]

	def read_long(self,unscale=False):
		val = struct.unpack('>L',self.port.read(4))
		self.checksum += val[0]
		self.checksum += (val[0]>>8)&0xFF
		self.checksum += (val[0]>>16)&0xFF
		self.checksum += (val[0]>>24)&0xFF

		if unscale:
			# Perform binary unscaling to get back to a float
			return val[0]/65536.0

		return val[0]

	def read_signed_long(self):
		val = struct.unpack('>l',self.port.read(4))
		self.checksum += val[0]
		self.checksum += (val[0]>>8)&0xFF
		self.checksum += (val[0]>>16)&0xFF
		self.checksum += (val[0]>>24)&0xFF
		return val[0]

	def write_byte(self,val):
		self.checksum += val
		return self.port.write(struct.pack('>B',val))

	def write_signed_byte(self,val):
		self.checksum += val
		return self.port.write(struct.pack('>b',val))

	def write_word(self,val):
		self.checksum += val
		self.checksum += (val>>8)&0xFF
		return self.port.write(struct.pack('>H',val))

	def write_signed_word(self,val):
		self.checksum += val
		self.checksum += (val>>8)&0xFF
		return self.port.write(struct.pack('>h',val))

	def write_long(self,val):
		self.checksum += val
		self.checksum += (val>>8)&0xFF
		self.checksum += (val>>16)&0xFF
		self.checksum += (val>>24)&0xFF
		return self.port.write(struct.pack('>L',val))

	def write_signed_long(self,val):
		self.checksum += val
		self.checksum += (val>>8)&0xFF
		self.checksum += (val>>16)&0xFF
		self.checksum += (val>>24)&0xFF
		return self.port.write(struct.pack('>l',val))

	def get_recvd_checksum(self):
		val = struct.unpack('>B',self.port.read(1))
		return val[0]&0x7F

	def create_checksum(self):
		return self.checksum&0x7F

# Command Enums

class Cmd():
	M1FORWARD = 0
	M1BACKWARD = 1
	SETMINMB = 2
	SETMAXMB = 3
	M2FORWARD = 4
	M2BACKWARD = 5
	M17BIT = 6
	M27BIT = 7
	MIXEDFORWARD = 8
	MIXEDBACKWARD = 9
	MIXEDRIGHT = 10
	MIXEDLEFT = 11
	MIXEDFB = 12
	MIXEDLR = 13
	GETM1ENC = 16
	GETM2ENC = 17
	GETM1SPEED = 18
	GETM2SPEED = 19
	RESETENC = 20
	GETVERSION = 21
	SETM1ENCCOUNT = 22
	SETM2ENCCOUNT = 23
	GETMBATT = 24
	GETLBATT = 25
	SETMINLB = 26
	SETMAXLB = 27
	SETM1PID = 28
	SETM2PID = 29
	GETM1ISPEED = 30
	GETM2ISPEED = 31
	M1DUTY = 32
	M2DUTY = 33
	MIXEDDUTY = 34
	M1SPEED = 35
	M2SPEED = 36
	MIXEDSPEED = 37
	M1SPEEDACCEL = 38
	M2SPEEDACCEL = 39
	MIXEDSPEEDACCEL = 40
	M1SPEEDDIST = 41
	M2SPEEDDIST = 42
	MIXEDSPEEDDIST = 43
	M1SPEEDACCELDIST = 44
	M2SPEEDACCELDIST = 45
	MIXEDSPEEDACCELDIST = 46
	GETBUFFERS = 47
	GETPWMS = 48
	GETCURRENTS = 49
	MIXEDSPEED2ACCEL = 50
	MIXEDSPEED2ACCELDIST = 51
	M1DUTYACCEL = 52
	M2DUTYACCEL = 53
	MIXEDDUTYACCEL = 54
	READM1PID = 55
	READM2PID = 56
	SETMAINVOLTAGES = 57
	SETLOGICVOLTAGES = 58
	GETMINMAXMAINVOLTAGES = 59
	GETMINMAXLOGICVOLTAGES = 60
	SETM1POSPID = 61
	SETM2POSPID = 62
	READM1POSPID = 63
	READM2POSPID = 64
	M1SPEEDACCELDECCELPOS = 65
	M2SPEEDACCELDECCELPOS = 66
	MIXEDSPEEDACCELDECCELPOS = 67
	SETM1DEFAULTACCEL = 68
	SETM2DEFAULTACCEL = 69
	SETPINFUNCTIONS = 74
	GETPINFUNCTIONS = 75
	SETDEADBAND = 76
	GETDEADBAND = 77
	RESTOREDEFAULTS = 80
	GETTEMP = 82
	GETTEMP2 = 83
	GETERROR = 90
	GETENCODERMODE = 91
	SETM1ENCODERMODE = 92
	SETM2ENCODERMODE = 93
	WRITENVM = 94
	READNVM = 95
	SETCONFIG = 98
	GETCONFIG = 99
	SETM1MAXCURRENT = 133
	SETM2MAXCURRENT = 134
	GETM1MAXCURRENT = 135
	GETM2MAXCURRENT = 136
	SETPWMMODE = 148
	GETPWMMODE = 149
	FLAGBOOTLOADER = 255
