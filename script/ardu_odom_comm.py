import rospy
import time
import math
import serial
import threading
import struct
import binascii

class ArduOdomCommunication:
	def __init__(self, port, baudrate, odom_freq, timeout):
		#===== serial config =====#
		self.port = port
		self.baudrate = baudrate
		self.odom_freq = odom_freq
		self.timeout = timeout
		#===== status flags =====#
		self._serialOK = False
		self._is_synced = False
		self._odom_new_data = False
		#self._cmd_new_data = False
		self._first_odom = True
		#self._first_cmd = True
		
		self.t_stop = threading.Event()			# threading stuff
		self.error_flag = False
		try:
			rospy.loginfo("Opening serial port: "+ self.port)
			self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
		except Exception:
			rospy.logerr("Failed to open serial port")
			raise
			return

		self.odom = {"pos":[0, 0, 0], "vel":[0, 0, 0]}
		self.cmd = [0, 0, 0]
		self.odom_seq = 0
		self.cmd_seq = 0
		self.last_odom_seq = 0
		self.last_cmd_seq = 0

	'''*******************************************************************
		Independent thread that constantly checking Serial port
	*******************************************************************'''
	def serial_thread(self):
		#===== Serial init =====#
		rospy.loginfo("===== Serial thread =====")
		rospy.loginfo("Rebooting Arduino...")
		time.sleep(1)    # for Arduino to reboot
		rospy.loginfo("First 3 data readings:")
		self.serial.reset_input_buffer()
		
		
		try:
			init_msg = self.serial.readline()
			for x in range(0, 3):
				init_msg = self.serial.readline()	# Note: try restart motor board if error
				print( init_msg.encode('ascii')[0:(len(init_msg)-1)] )
		except Exception:
			rospy.logerr("Port %s timeout after %d seconds", self.port, self.timeout)
			self.serial.close()
			raise
			return
		
		#===== Send starting signal =====#
		self._serialOK = True
		rospy.loginfo("Sending starting signal \"B\"...")
		self.serial.write( "BB".encode('ascii') )
		init_msg = self.serial.readline()	# clear waiting 4

	
		#===== Start packet rx + parsing loop =====#
		while (not self.t_stop.is_set()):
			try:
				reading = self.serial.read(2)
			except Exception:
				self.error_flag = True
				break

			### encoder data packet
			if reading[0] == '\x00' and reading[1] == '\xFF':
				try:
					ser_in = self.serial.read(30)
				except Exception:
					self.error_flag = True
					break
					
				self.odom_decode(ser_in, 30)
				self._is_synced = True	
				#== debug
				#toHex = lambda x: "".join("{:02X}".format(ord(c)) for c in reading)
				#print(toHex(b'\x03\xac23\n'))
			
			#we have only one packet type now
			#========= command data packet =========#
			#elif reading[0] == '\xFF' and reading[1] == '\xFC':
			#	ser_in = self.serial.read(13)
			#	self.cmd_decode(ser_in, 13)
			#	self._is_synced = True

			### lost sync
			else:
				if self._is_synced:
					if self._first_odom:
						rospy.loginfo("Initial syncing...")
						self._is_synced = False
						continue
					rospy.logerr('Out of sync:')
				toHex = lambda x: "".join("{:02X}".format(ord(c)) for c in reading)
				print(toHex(b'\x03\xac23\n'))

				bfr = self.serial.read(1)
				toHex = lambda x: "".join("{:02X}".format(ord(c)) for c in bfr)
				print(toHex(b' '))
				self._is_synced = False
				
					
		
		if self.error_flag:
			rospy.logerr('serial read error')
			self.serial.write( 'RRR'.encode('ascii') )
			self.serial.close()
			self._serialOK = False
			self._is_synced = False
			self._odom_new_data = False
			self._cmd_new_data = False
			print("thread ends")
			raise
			return
		
		#===== if thread stop is set =====#
		print("Sending stop signal to motor control")
		self.serial.write( 'R'.encode('ascii') )
		self.serial.close()
		self._serialOK = False
		self._is_synced = False
		self._odom_new_data = False
		self._cmd_new_data = False
		print("thread ends")

	'''*******************************************************************
		Decode odometry data
	*******************************************************************'''
	def odom_decode(self, data, size):
		#https://docs.python.org/3/library/struct.html
		self.odom_seq = struct.unpack('<H', data[0:2])[0]	# unsigned short
		self.odom_ut  = struct.unpack('<I', data[2:6])[0]	# unsigned int 4B
		self.odom["pos"][0] = struct.unpack('<f', data[6:10])[0] 	# float 4B
		self.odom["pos"][1] = struct.unpack('<f', data[10:14])[0]
		self.odom["pos"][2] = struct.unpack('<f', data[14:18])[0]
		self.odom["vel"][0] = struct.unpack('<f', data[18:22])[0] 	# float 4B
		self.odom["vel"][1] = struct.unpack('<f', data[22:26])[0]
		self.odom["vel"][2] = struct.unpack('<f', data[26:30])[0]
		
		#debug
		#print("odom", self.odom_seq, self.odom[0:3])
		if (self.odom_seq != ((self.last_odom_seq + 1)%65536) ):
			if not self._first_odom:
				rospy.logwarn("odom seq mismatch, prev: %d, now:%d", self.last_odom_seq, self.odom_seq)				
		if self._first_odom:
			rospy.loginfo("Got first data! Now publishing!")
			self._first_odom = False
		self.last_odom_seq = self.odom_seq
		self._odom_new_data = True
	
	'''*******************************************************************
		NO CMD PACKET AT THE MOMENT
	*******************************************************************
	def cmd_decode(self, data, size):
		#https://docs.python.org/3/library/struct.html
		self.cmd[0] = struct.unpack('>f', data[0:4])[0] 	# int 4B
		self.cmd[1] = struct.unpack('>f', data[4:8])[0]
		self.cmd[2] = struct.unpack('>f', data[8:12])[0]
		self.cmd_seq = struct.unpack('B', data[12:13])[0]	# unsigned byte	
		
		#debug
		#print("cmdA", self.cmd[0], "cmdB", self.cmd[1], "cmdC", self.cmd[2])
		if (self.cmd_seq != ((self.last_cmd_seq + 1)%256) ):
			if not self._first_cmd:
				rospy.logwarn("cmd seq mismatch, prev: %d, now:%d", self.last_cmd_seq, self.cmd_seq)
			else:
				self._first_cmd = False
		self.last_cmd_seq = self.cmd_seq
		self._cmd_new_data = True
		self._pos_new_data = True'''

		
	'''*******************************************************************
		Module communication from outside
	*******************************************************************'''		
	def serialOK(self):
		return self._serialOK
		
	def odom_new_data(self):
		return self._odom_new_data

	def get_odom_data(self):
		if self._odom_new_data:
			# data assign
			self._odom_new_data = False
			return {"seq":self.odom_seq, "micro_s": self.odom_ut,
					"pos":self.odom["pos"], "vel":self.odom["vel"] }
		else:
			return None
		
	''' NO CMD PACKET AT THE MOMENT
	def get_cmd_data(self):
		if self._cmd_new_data:
			self._cmd_new_data = False
			return {"seq":self.cmd_seq, "cmd":self.cmd}
		else:
			return None'''
		
	'''*******************************************************************
		send vel_cmd to vehicle
	*******************************************************************'''
	def send_dir_cmd(self, wheel):
		if self._serialOK:
			if wheel["l"] > 0:
				if wheel["r"] >0:
					dir_cmd = 'W'
				elif  wheel["r"] < 0:
					dir_cmd = 'D'
				else:
					dir_cmd = 'E'
			elif wheel["l"] < 0:
				if wheel["r"] >0:
					dir_cmd = 'A'
				elif  wheel["r"] < 0:
					dir_cmd = 'S'
				else:
					dir_cmd = 'Z'
			else:
				if wheel["r"] >0:
					dir_cmd = 'Q'
				elif  wheel["r"] < 0:
					dir_cmd = 'C'
				else:
					dir_cmd = ' '
			# debug
			#print(binascii.hexlify(serial_cmd))
			self.serial.write( dir_cmd )	
	
	def stopThread(self):
		self.t_stop.set()
		if self._serialOK:
			while self._serialOK:
				time.sleep(0.1)		# wait for thread to stop
			self.serial.close()