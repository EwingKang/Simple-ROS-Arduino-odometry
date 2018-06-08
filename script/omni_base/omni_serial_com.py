import rospy

import time
import sys
import math
import serial
import threading
import struct
import binascii

class OmniSerialCom:
	def __init__(self, port, baudrate, imu_freq, odom_freq, timeout):
	
		self.port = port
		self.baudrate = baudrate
		self.imu_freq = imu_freq
		self.odom_freq = odom_freq
		self.timeout = timeout
		
		self._serialOK = False

		self.is_synced = False
		self._imu_new_data = False
		self._odom_new_data = False
		self._cmd_new_data = False
		self._first_odom = True
		self._first_cmd = True
		
		self.t_stop = threading.Event()
		try:
			rospy.loginfo("Opening serial port: "+ self.port)
			self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
		except:
			rospy.logerr("serial err")
			raise Exception('error')
			return

		self.imu = {"accel":[0, 0, 0], "gyro":[0, 0, 0]}
		self.odom = [0, 0, 0]
		self.cmd = [0, 0, 0]
		self.odom_seq = 0
		self.cmd_seq = 0
		self.last_odom_seq = 0
		self.last_cmd_seq = 0


	'''*******************************************************************
		Independent thread that constantly checking Serial port
	*******************************************************************'''
	def serial_thread(self):
		# Serial initialization
		rospy.loginfo("===== Serial thread =====")
		try:
			rospy.loginfo("First 3 data readings:")
			self.serial.reset_input_buffer()
			init_msg = self.serial.readline()
			for x in range(0, 3):
				init_msg = self.serial.readline()	### Note: try restart motor board if error ###
				print( init_msg.encode('ascii')[0:(len(init_msg)-1)] )
		except serial.serialutil.SerialException:
			rospy.logerr("Port timeout after %d seconds at: %s", self.timeout, self.port)
			self.serial.close
			raise Exception('timeout')
			return
		
		# sent start signal
		self._serialOK = True
		rospy.loginfo("Sending starting signal \"SSSS\"")
		self.serial.write( "SSSS".encode('ascii') )
		# time.sleep(0.01)		# for Arduino to reboot
		
		# continuous packet recieve
		while (not self.t_stop.is_set()): 
			reading = self.serial.read(2)
			
			#========= imu data packet =========#
			if reading[0] == '\xFF' and reading[1] == '\xFA':		
				ser_in = self.serial.read(12)
				self.imu_decode(ser_in, 12)
				self.is_synced = True
				#debug
				#toHex = lambda x: "".join("{:02X}".format(ord(c)) for c in reading)
				#print(toHex(b'\x03\xac23\n'))
				
			#========= encoder data packet =========#
			elif reading[0] == '\xFF' and reading[1] == '\xFB':
				ser_in = self.serial.read(7)
				self.odom_decode(ser_in, 7)
				self.is_synced = True
			
			#========= command data packet =========#
			elif reading[0] == '\xFF' and reading[1] == '\xFC':
				ser_in = self.serial.read(13)
				self.cmd_decode(ser_in, 13)
				self.is_synced = True
				
			#=========  lost sync =========#
			else:
				rospy.logerr('Out of sync:')
				toHex = lambda x: "".join("{:02X}".format(ord(c)) for c in reading)
				print(toHex(b'\x03\xac23\n'))

				bfr = self.serial.read(1)
				toHex = lambda x: "".join("{:02X}".format(ord(c)) for c in bfr)
				print(toHex(b' '))
				self.is_synced = False

		# if threads ends here
		print("Sending stop signal to motor control")
		self.serial.write( 'R'.encode('ascii') )
		self._serialOK = False
		self.is_synced = False
		self._odom_new_data = False
		self._imu_new_data = False
		self._cmd_new_data = False
		print("thread ends")
	
	
	'''*******************************************************************
		Decode imu data
	*******************************************************************'''
	def imu_decode(self, data, size):
		#https://docs.python.org/3/library/struct.html
		self.imu["accel"][0] = struct.unpack('>h', data[0:2])[0] 		# signed short, 2B
		self.imu["accel"][1] = struct.unpack('>h', data[2:4])[0]
		self.imu["accel"][2] = struct.unpack('>h', data[4:6])[0]
		self.imu["gyro"][0] = struct.unpack('>h', data[6:8])[0]
		self.imu["gyro"][1] = struct.unpack('>h', data[8:10])[0]
		self.imu["gyro"][2] = struct.unpack('>h', data[10:12])[0]
		#debug
		#print("imu", self.seq, " t_micro:", self.t_micro)
		self._imu_new_data = True
	
	'''*******************************************************************
		Decode odometry data
	*******************************************************************'''
	def odom_decode(self, data, size):
		#https://docs.python.org/3/library/struct.html
		self.odom[0] = struct.unpack('>h', data[0:2])[0] 	# signed short 2B
		self.odom[1] = struct.unpack('>h', data[2:4])[0]
		self.odom[2] = struct.unpack('>h', data[4:6])[0]
		self.odom_seq = struct.unpack('B', data[6:7])[0]	# unsigned byte	
		
		#debug
		#print("odom", self.odom_seq, self.odom[0:3])
		if (self.odom_seq != ((self.last_odom_seq + 1)%256) ):
			if not self._first_odom:
				rospy.logwarn("odom seq mismatch, prev: %d, now:%d", self.last_odom_seq, self.odom_seq)
			else:
				self._first_odom = False	
		self.last_odom_seq = self.odom_seq
		self._odom_new_data = True
	
	'''*******************************************************************
		Decode 5hz data
	*******************************************************************'''
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
		self._pos_new_data = True

	'''*******************************************************************
		Module communication from outside
	*******************************************************************'''		
	def serialOK(self):
		if self._serialOK:		# and self.is_synced
			return True
		else:
			return False
		
	def imu_new_data(self):
		return self._imu_new_data
		
	def odom_new_data(self):
		return self._odom_new_data
	
	def cmd_new_data(self):
		return self._cmd_new_data
	
	def pos_new_data(self):
		return self._pos_new_data
		
	def get_imu_data(self):
		if self._imu_new_data:
			# data assign
			self._imu_new_data = False
			return self.imu
		else:
			return None
		
	def get_odom_data(self):
		if self._odom_new_data:
			# data assign
			self._odom_new_data = False
			return {"seq":self.odom_seq, "pos_dt":self.odom}
		else:
			return None
		
	def get_cmd_data(self):
		if self._cmd_new_data:
			self._cmd_new_data = False
			return {"seq":self.cmd_seq, "cmd":self.cmd}
		else:
			return None
		
	'''*******************************************************************
		send vel_cmd to vehicle
	*******************************************************************'''
	def send_vel_cmd(self, veh_cmd):
		if self._serialOK:
			serial_cmd = bytearray(b'\xFF\xFE')
			serial_cmd.append(0x01)			# base vector mode
			
			clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
			serial_cmd += struct.pack('>h',clamp( veh_cmd[0], -37268, 32767 ) ) #2-bytes 
			serial_cmd += struct.pack('>h',clamp( veh_cmd[1], -37268, 32767 ) )
			serial_cmd += struct.pack('>h',clamp( veh_cmd[2], -37268, 32767 ) )
			
			#debug
			#print(binascii.hexlify(serial_cmd))

			self.serial.write( serial_cmd )	
	
	def stopThread(self):
		self.t_stop.set()
		if self._serialOK:
			while self._serialOK:
				time.sleep(0.1)		# wait for thread to stop
			self.serial.close()