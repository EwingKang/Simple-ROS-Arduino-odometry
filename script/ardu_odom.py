import rospy
import tf
import time
import sys
import math
from operator import add
import string
import threading
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class ArduOdom:
	'''*******************************************************************
			Constructor
	*******************************************************************'''
	def __init__(self):
		#======== Get params =========#
		self.param = {}			# create a dictionary
		#self.debug_mode = bool(rospy.get_param('~debug_mode', False)) # true for detail info        
			
		self.param["odom_topic"] = rospy.get_param("~odom_topic", 'odom')
		self.param["baseId"] = rospy.get_param('~base_id', 'base_link') # base frame id
		self.param["odomId"] = rospy.get_param('~odom_id', 'odom') # odom frame id
		self.param["enable_tf"] = rospy.get_param("~enable_tf", True)
		
		self.param["device_port"] = rospy.get_param('~port', '/dev/ttyACM0') # port
		self.param["baudrate"] = int( rospy.get_param('~baudrate', '115200') ) # baudrate
		self.param["timeout"] = float( rospy.get_param('~serial_timeout', '10') ) #
		self.param["odom_freq"] = float( rospy.get_param('~odom_freq', '100') ) # hz of communication
		self.param["tx_freq"] = float( rospy.get_param('~tx_freq', '5') )      # hz of communication
		
		self.param["quadrature"] = rospy.get_param("~quadrature", True)
		self.param["vel_cmd"] = rospy.get_param("~cmd_vel", 'cmd_vel')
		self.param["cmd_timeout"] = float( rospy.get_param('~cmd_vel_timeout', '3') ) #
	
		
		rospy.set_param("serial_odom", self.param)		# set ros param
		
		#========== ROS message ==========#
		rospy.loginfo("Initiating Node")
		rospy.loginfo("Publishing odometry at: \"" + self.param["odom_topic"] + "\"")
				
		self.comm_handle_ok = False		# set before timer to prevent stuff from starting prematurely
		
		#========== ROS handler ==========#
		if not self.param["quadrature"]:
			self.enc_sub = rospy.Subscriber(self.param["vel_cmd"], Twist, self.cmdvel_cb, queue_size=10) 
			rospy.loginfo("Non-quadrature type encoder,subscribing to \"" + self.param["vel_cmd"] + "\" for wheel direction determination")
		self.odom_pub = rospy.Publisher(self.param["odom_topic"], Odometry, queue_size=10)		# pubisher

		self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.param["odom_freq"]), self.odomPub) # timer
		self.timer_txcmd = rospy.Timer( rospy.Duration(1.0/self.param["tx_freq"]),
																	self.send_vel_cmd) # transmit command
		
		if self.param["enable_tf"]:
			rospy.loginfo("Publishing transform from %s to %s", self.param["odomId"], self.param["baseId"])
			self.tf_br = tf.TransformBroadcaster()

		#========== variables for ROS topic ==========#
		self.odom = Odometry()
		self.odom.header.seq = 0
		self.odom.header.frame_id = self.param["odomId"]
		self.odom.child_frame_id = self.param["baseId"]
		for i in range(36):
			self.odom.pose.covariance[i] = 0
			self.odom.twist.covariance[i] = 0
		self.odom.twist.covariance[0] = 1.0
		self.odom.twist.covariance[35] = 1.0
	
		self.x_e = float(0)
		self.y_e = float(0)
		self.th = float(0)
		self.odom_last_seq = 0;
		self.enc_count_per_revo = (390*4)
		self.wheel_radius = 0.029
		self.b = 0.1 						# wheel separation
		self.last_odom_time = time.time()
		self.last_cmd_vel_time = self.last_odom_time
		self.no_cmd_received = True
		self.first_odom = True
		
		self.veh_cmd = {"Vx":0, "Vy":0, "Omega":0}
	
	'''*******************************************************************
		Get serial handle object, must be called before every functions 
		except constructor
	*******************************************************************'''
	def set_data_port(self, _serial_handle):
		self.comm_handle = _serial_handle
		self.comm_handle_ok = True

	'''*******************************************************************
		ROS cmd_vel Subscriber callback
	*******************************************************************'''
	def cmdvel_cb(self, vel_cmd):
		self.veh_cmd["Vx"] = vel_cmd.linear.x
		self.veh_cmd["Vy"] = vel_cmd.linear.y
		self.veh_cmd["Omega"] = vel_cmd.angular.z
		self.no_cmd_received = False
		self.last_cmd_vel_time = time.time()
		
	'''*******************************************************************
		send velocity command to serial handler
	*******************************************************************'''
	def send_vel_cmd(self, event):
		### if haven't seen vel_cmd for a long time
		if( (time.time()-self.last_cmd_vel_time) > self.param["cmd_timeout"] and 
														not self.no_cmd_received ):
			self.veh_cmd["Vx"] = 0
			self.veh_cmd["Vy"] = 0
			self.veh_cmd["Omega"] = 0
			self.no_cmd_received = True
			rospy.logwarn("no topic \"%s\" received, base stop.", self.param["vel_cmd"])
		### decode cmd_vel
		wheel = {"l":0, "r":0}
		wheel["l"] = self.veh_cmd["Vx"] - self.veh_cmd["Omega"] * self.b/2 # vL = vx - omega*(b/2)
		wheel["r"] = self.veh_cmd["Vx"] + self.veh_cmd["Omega"] * self.b/2 # vR = vx + omega*(b/2)
		self.comm_handle.send_dir_cmd(wheel)

	'''*******************************************************************
		ROS Odometry topic publisher, called by timer
	*******************************************************************'''	
	def odomPub(self, event):
		if self.comm_handle_ok and self.comm_handle.serialOK():
			if self.comm_handle.odom_new_data():
				#===== get new data from comm handle =====#
				xyt_rtn = self.comm_handle.get_odom_data()
				if xyt_rtn is None:
					return

				#===== continuity check =====#
				if (xyt_rtn["seq"] != ((self.odom_last_seq + 1)%65536) ):
					if (not self.first_odom):
						rospy.logwarn("callback sequence mismatch, prev: %d, now:%d", 
														self.odom_last_seq, xyt_rtn["seq"] )
						self.odom_last_seq = xyt_rtn["seq"]
						return		# disregard first data
					else:
						self.first_odom = False
						self.odom_last_seq = xyt_rtn["seq"]
						return		# disregard first data
				self.odom_last_seq = xyt_rtn["seq"]

				#===== data assign =====#
				### DEBUG
				#self.temp += xyt_rtn["enc_dt"][1]
				#print(self.temp)
				#print(" A: {} B: {} C: {}".format( xyt_rtn["enc_dt"][0], 
				#								   xyt_rtn["enc_dt"][1], 
				#								   xyt_rtn["enc_dt"][2]  ) )
				self.x_e = xyt_rtn["pos"][0]
				self.y_e = xyt_rtn["pos"][1]
				self.th = xyt_rtn["pos"][2]
				vx = xyt_rtn["vel"][0]
				vy = xyt_rtn["vel"][1]
				omega =  xyt_rtn["vel"][2]

				time_now = time.time()
				dt = time_now - self.last_odom_time
				
				#======= ROS topic publish =======#
				self.odom.header.seq += 1
				self.odom.header.stamp = rospy.Time.now()
				self.odom.pose.pose.position = geometry_msgs.msg.Point(self.x_e, self.y_e, 0.0)
				quat = tf.transformations.quaternion_from_euler(0, 0, self.th);
				self.odom.pose.pose.orientation.x = quat[0]
				self.odom.pose.pose.orientation.y = quat[1]
				self.odom.pose.pose.orientation.z = quat[2]
				self.odom.pose.pose.orientation.w = quat[3]
				self.odom.twist.twist.linear = Vector3(vx, vy, 0.0)
				self.odom.twist.twist.angular = Vector3(0.0, 0.0, omega)
				
				self.odom_pub.publish(self.odom)
				
				if self.param["enable_tf"]:
					self.tf_br.sendTransform((self.x_e, self.y_e, 0),
									quat,
									self.odom.header.stamp,
									self.param["baseId"],
									self.param["odomId"]			)
				
				self.last_odom_time = time_now;
			#else:
				#rospy.logerr('data not available')