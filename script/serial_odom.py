#!/usr/bin/python

import rospy
import tf
import time
import sys
import math
import serial
import struct
import string
import threading
import geometry_msgs.msg
from geometry_msgs.msg import Vector3
#from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class SerialOdom:
	def __init__(self):
		# Get params
		
		self.odom_topic = rospy.get_param("~odom_topic", '/odom')
		self.baseId = rospy.get_param('~base_id', 'base_link') # base frame id
		self.odomId = rospy.get_param('~odom_id', 'odom') # odom frame id
		self.enable_tf = rospy.get_param("~enable_tf", True)
		self.device_port = rospy.get_param('~port', '/dev/ttyACM0') # port
		self.baudrate = int( rospy.get_param('~baudrate', '57600') ) # baudrate
		self.comm_freq = float( rospy.get_param('~odom_freq', '10') ) # hz of communication
		#self.wheelSep = float( rospy.get_param('~wheel_separation', '0.158') ) # unit: meter 
		#self.wheelRad = float( rospy.get_param('~wheel_radius', '0.032') ) # unit: meter
		#self.VxCov = float( rospy.get_param('~vx_cov', '1.0') ) # covariance for Vx measurement
		#self.VyawCov = float( rospy.get_param('~vyaw_cov', '1.0') ) # covariance for Vyaw measurement
		#self.debug_mode = bool(rospy.get_param('~debug_mode', False)) # true for detail info        
		self.timeout = float( rospy.get_param('~timeout', '10') ) # hz of odom pub
		
		rospy.set_param("~odom_topic", self.odom_topic)
		rospy.set_param("~base_id", self.baseId)
		rospy.set_param("~odom_id", self.odomId)
		rospy.get_param("~enable_tf", self.enable_tf)
		rospy.get_param("~port", self.device_port)
		rospy.get_param("~baudrate", self.baudrate)
		rospy.get_param("~odom_freq", self.comm_freq)
		
		rospy.loginfo("Publishing odometry: " + self.odom_topic)

		# ROS handler        
		self.encsub = rospy.Subscriber('encoder', Vector3, self.wheelcmdSub, queue_size=10)
		self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)   
		self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.comm_freq), self.odomPub) 
		self.serialWriter = rospy.Timer(rospy.Duration(0.1), self.msgSendCB) # 10Hz
		if self.enable_tf:
			rospy.loginfo('Publishing transform from %s to %s', self.odomId, self.baseId)
			self.tf_br = tf.TransformBroadcaster()

		# variable
		self.odom = Odometry()
		self.odom.header.seq = 0
		self.odom.header.frame_id = self.odomId
		self.odom.child_frame_id = self.baseId
		for i in range(36):
			self.odom.pose.covariance[i] = 0
			self.odom.twist.covariance[i] = 0
		self.odom.twist.covariance[0] = 1.0
		self.odom.twist.covariance[35] = 1.0
		
		self.seq = 0
		self.lastSeq = 0
		self.t_micro = 0
		self.x = 0
		self.y = 0
		self.th = 0
		self.v_x = 0
		self.v_y = 0
		self.omega = 0

		self.serial_cmd = ' '		# all stop
		self.dataOK = False
		self.serialOK = False
		self.serial = serial.Serial()
		self.t_stop = threading.Event()
		try:
			rospy.loginfo("Starting on serial port: "+ self.device_port)
			self.serial = serial.Serial(self.device_port, self.baudrate, timeout=self.timeout)
		except:
			rospy.logerr("serial err")
			self.serial.close
			sys.exit(0)

		# reading loop 
	def serial_handle(self):
		# Serial initialization
		try:
			self.serial.reset_input_buffer()
			rospy.loginfo("Reaching for serial")
			rospy.loginfo("Here are the first 5 data readings ...")
			time.sleep(1)
			self.serial.reset_input_buffer()
			init_msg = self.serial.readline()
			for x in range(0, 5):
				#init_msg = self.serial.read(10)
				init_msg = self.serial.readline()
				rospy.loginfo( init_msg.encode('ascii')[0:(len(init_msg)-1)] )
		except serial.serialutil.SerialException:
			rospy.logerr("Port timeout after %d seconds at: %s", self.timeout, self.device_port)
			self.serial.close
			sys.exit(0)
		
		# sent start signal
		self.serialOK = True
		self.serial.write( 'B'.encode('ascii') )
		time.sleep(0.08)		# for Arduino to reboot
		# continuous packet recieve
		while (not self.t_stop.is_set()): 
			reading = self.serial.read(32)
			if reading[0] == '\0' and reading[1] == '\xFF':
				self.data = reading
				self.dataOK = True
				#debug
				#toHex = lambda x: "".join("{:02X}".format(ord(c)) for c in reading)
				#print(toHex(b'\x03\xac23\n'))
			else:
				# lost sync
				rospy.logerr('Out of sync:')
				toHex = lambda x: "".join("{:02X}".format(ord(c)) for c in reading)
				print(toHex(b'\x03\xac23\n'))

				self.serial.read(1)
				self.dataOK = False
				
		self.serial.write( 'B'.encode('ascii') )
		time.sleep(0.08)
		print("thread ends")
				
	def stopThread(self):
		self.t_stop.set()
				
	def decode(self):
		if self.dataOK :
			#https://docs.python.org/3/library/struct.html
			self.seq = struct.unpack('H', self.data[2:4])[0] 		# unsigned int 2B
			self.t_micro = struct.unpack('I', self.data[4:8])[0] 	# unsigned long 4B, arduino Micros()
			self.x = struct.unpack('f', self.data[8:12])[0] 			# float 4B
			self.y = struct.unpack('f', self.data[12:16])[0]
			self.th = struct.unpack('f', self.data[16:20])[0]
			self.v_x = struct.unpack('f', self.data[20:24])[0]
			self.v_y = struct.unpack('f', self.data[24:28])[0]
			self.omega = struct.unpack('f', self.data[28:32])[0]
			
			#debug
			#print("seq:", self.seq, " t_micro:", self.t_micro)

	def wheelcmdSub(self, enc):
		vel_left = enc.x 
		vel_right = enc.y
		if vel_left > 0:
			if vel_right >0:
				self.serial_cmd = 'W'
			elif  vel_right < 0:
				self.serial_cmd = 'D'
			else:
				self.serial_cmd = 'E'
		elif vel_left < 0:
			if vel_right >0:
				self.serial_cmd = 'A'
			elif  vel_right < 0:
				self.serial_cmd = 'S'
			else:
				self.serial_cmd = 'Z'
		else:
			if vel_right >0:
				self.serial_cmd = 'Q'
			elif  vel_right < 0:
				self.serial_cmd = 'C'
			else:
				self.serial_cmd = ' '
		
	def odomPub(self, event):
		if self.serialOK:
			if self.dataOK:
				self.decode()
				if (self.seq != (self.lastSeq + 1) ):
					rospy.logwarn("Income sequence mismatch, prev: %d, now:%d", self.lastSeq, self.seq)
				
				self.odom.header.seq += 1
				self.odom.header.stamp = rospy.Time.now()
				# odom.pose is a PoseWithCovariance.msg
				self.odom.pose.pose.position = geometry_msgs.msg.Point(self.x, self.y, 0.0)
				quat = tf.transformations.quaternion_from_euler(0, 0, self.th);
				self.odom.pose.pose.orientation.x = quat[0]
				self.odom.pose.pose.orientation.y = quat[1]
				self.odom.pose.pose.orientation.z = quat[2]
				self.odom.pose.pose.orientation.w = quat[3]
				

				# odom.twist is a TwistWithCovariance.msg
				self.odom.twist.twist.linear = Vector3(self.v_x, self.v_y, 0.0)
				self.odom.twist.twist.angular = Vector3(0.0, 0.0, self.omega)
				
				self.pub.publish(self.odom)
				
				if self.enable_tf:
					self.tf_br.sendTransform((self.x, self.y, 0),
									quat,
									self.odom.header.stamp,
									self.baseId,
									self.odomId			)
				self.dataOK = False
				self.lastSeq = self.seq
			#else:
				#rospy.logerr('data not available')
			
	def msgSendCB(self, event):
		if self.serialOK:
			self.serial.write( self.serial_cmd.encode('ascii') )
		
if __name__ == "__main__":
   
	# ROS Init    
	rospy.init_node('serial_odom', anonymous=True, disable_signals=True)
	
	# Construct BaseControl Obj
	rospy.loginfo("Serial odometry starting")
	so = SerialOdom()
	thread = threading.Thread(target=so.serial_handle)
	#thread.daemon=True
	thread.start()
	rospy.spin()
	print("Shutting down")
	so.stopThread()
	thread.join()
	so.serial.close()     
	rospy.signal_shutdown("user ends")
	'''except KeyboardInterrupt:
		print("Shutting down")
		so.stopThread()
		thread.join()
		so.serial.close()     
		rospy.signal_shutdown()
	'''
