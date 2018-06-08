#!/usr/bin/python
import rospy
import sys
import threading
import math

from ardu_odom import ArduOdom
from ardu_odom_comm import ArduOdomCommunication

if __name__ == "__main__":
   	rospy.init_node('serial_odom', anonymous=True, disable_signals=True)	# ROS node init
	node = ArduOdom()		# using node object
	rospy.loginfo("Arduino serial odometry starting...")
	
	# try to start a serial communication object (open com port)
	try:
		omni_com = ArduOdomCommunication( port      = node.param["device_port"],
										  baudrate  = node.param["baudrate"]   ,
										  odom_freq = node.param["odom_freq"]  ,
										  timeout   = node.param["timeout"]      )
	except:
		print("Fail to create serial connection, shutting down")
		rospy.signal_shutdown("open com port error")
		sys.exit(1)
	
	# try to start a thread for serial comm handling
	try:
		thread = threading.Thread(target=omni_com.serial_thread)
		#thread.daemon=True
		thread.start()
	except:
		rospy.signal_shutdown("serial thread error")
		omni_com.stopThread()
		sys.exit(1)

	node.set_data_port(omni_com)
	rospy.spin()
	
	print("SIGINT, stopping...")
	omni_com.stopThread()
	thread.join()
	rospy.signal_shutdown("user ends")
	print("Node ends")
