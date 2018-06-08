# Simple-ROS-Arduino-odometry
I started this project because I'm done with rosserial: it is just too bulky for the arduino's 2KB flash, and almost impossible to debug. The only two things it does is refuse to connect or report trying to sync. Trust me, it's much quicker to just write your own serial communication. It's quicker even I've never written any Python and actually learning Python at the same time!  
    For more information, please see [About the Code](#project-details) section below.
    
**PLEASE CHECK [ENCODER](#encoder) SECTION BELOW BEFORE PROCEEDING**
    
  
## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites
Tested and built with ROS-kinetic, Python 2.7, with Ubuntu 16.04LTS, but should work with any version near enough.  
Please make sure you have the following items set-up or at least you know what you're doing: 
* [install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) (tested in ROS-kinetic) 
* [setup ROS catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) (it is recommanded to source the setup in .bashrc) 
* [Install the Arduino IDE](https://www.arduino.cc/en/Guide/Linux) ( I recommand you to put the Arduino IDE under /opt/ before running install.sh )
* Add your user to dialout group as required by the Arduino IDE (Hint1:`sudo usermod -a -G dialout yourUserName`, Hint2: you have to login again, just reboot!)
* (**For non-quadrature encoders aka tachometers**) We need to subscribe to topic `/cmd_vel`(_[geometry_msgs.msg::Twist](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html)_) to determine the direction of the wheel rotation.

### Installation

1. Go to your catkin directory, clone the project into folder /serial_odom along with all the submodules(Arduino library).
   ```
   cd ~/catkin_ws/src
   git clone --recurse-submodules -j8  https://github.com/EwingKang/Simple-ROS-Arduino-odometry serial_odom
   ```
2. (**Optional**)Link(or move) the Ardunio program to workspace. It is possible to open the (project)/arduino_files/serial_odom/serial_odom.ino with Arduino IDE directly. But I find it faster to find if everything is under Arduino IDE directory. You'll see a link folder in the Arduino/ afterwards
   ```
   ln -s ~/catkin_ws/src/serial_odom/arduino_files/serial_odom/ ~/Arduino/
   ```
3. "Compile" the file with catkin_make so ros recgonize the package
   ```
   cd ~/catkin_ws/
   catkin_make
   ```
4. Upload the Arduino code with Arduino IDE
    Please [setup your vehicle parameter](vehicle_settings) first!

### Vehicle settings
To make sure you have the correct result, it is necessary to setup parameters matching your vehicle geometry. Since odometry is calculated "on-board" the Arduino, you should change the config header accordingly

* Physical parameters in [vehicle_config.h](arduino_files/serial_odom/vehicle_config.h):
    ```
    #define ENC_TYPE 4
    #define ENC_ENABLE_CUMU		// enable cumulative counter

   //========== VEHICLE CONFIG ==========//
    #define WHEEL_RAD 0.033f		// wheel radius in m
    #define B 0.1f					// b = wheel separation distance in m
    #define ENC_REDUCTION 21		// encoder reduction ratio = (wheel rpm)/(encoder rpm)
    #define MAX_WHEEL_RPS ((float)201/(float)60)	// maximum wheel revolution per second
    
    //========== ENCODER CONFIG ==========//
    #define ENC_TPR 334				// encoder tooth count
    #define WHEEL_PPR (ENC_TYPE*ENC_TPR*ENC_REDUCTION)	// pulse count per wheel revolution
    ```  
* Special (cheap/homemade) Encoders:
    Please check the  [Tachometer](tachometer) section below.

### Run the node
Hook up all your cable, make sure you have the correct device (in this case /dev/ttyACM0), and simply rosrun:
```
rosrun serial_odom serial_odom.py _port:=/dev/ttyACM0
```
The code will reset the Arduino automatically, and you'll see:
1. Node starting
2. Independent communication thread starting
3. Rebooting Arduino
4. Arduino standby message
2. Signal to start Arduino
6. First packet received and `/odom` is now published

### Check the result
In another terminal, you can check the result by:
* rostopic
    You be able to see the `/odom` topic with `rostopic list`. Use `rostopic echo /odom` to see the raw data
* rviz:
    Run `rviz` and the ROS visualization tool will begin. Set the frame to "_Odom_" and you'll see "_base_link_" moving around.

### ROS node parameters
All the parameters are under namespace `/serial_odom`
* ~odom_topic (default: 'odom')
* ~base_id' (default: 'base_link') # base frame id
* ~odom_id' (default: 'odom') # odom frame id
* ~enable_tf" (default: True)

* ~port' (default: '/dev/ttyACM0') # port
* ~baudrate' (default: '115200') ) # baudrate
* ~serial_timeout' (default:'10') ) 
* ~odom_freq' (default: '100') ) # hz of communication*
* ~tx_freq' (default: '5') )      # hz of communication
		
* ~quadrature", True)
* ~cmd_vel", 'cmd_vel')
* ~cmd_vel_timeout', '3') ) #


## Project details  
TODO
### Tachometer
TODO
### Encoder

I'm using a cheap ass 175NTD(~5$) chinese-made encoder like [this one](https://world.taobao.com/item/9610181079.htm?spm=a21wu.10013406-tw.0.0.3ca71191qiAk3A) and it is super lame. Despite the claimed AB phase output with a 100 grid encoder plate, it just simply connot hangle that. And I have to make my own 25 grid encoder plate out of catdboard. Anyway, you're welcome to take this code and modified it to adopt proper A/B phase encoder, or even better, you can donate me some proper encoders and I'll change the code for you. Currectly, the direction information comes from a ROS topic, which piggyback the motor controller.

### Python
I'm using Python 2.7 that comes with Ubuntu-16.04 & ROS-kinetic, with following modules:
* rospy and its peripherals
* rostf
* sys, time, math
* serial ([pySerial](http://pyserial.readthedocs.io/en/latest/pyserial.html))
* threading

pySerial is used to easily handle the serial port and capture the data. Serial communication is handled in a seperate thread wih Python module "threading", which make it very elastic, powerful, and stable.  

### Arduino
On the Arduino side, data is struted and can be easily modified. [0x00 0xFF] is used as the starting signal of each packet.
 
### Line-by-line explaination
TODO: see wiki

## Other costumizable parameters
* Arduino sceduling system  
    ```
    #define SERIAL_PERD 100000  // Serial tx/rx check period, 100 ms is 10 Hz
    #define CAR_PERD 20000	    // Main calculation period, 20 ms is 50 Hz
    ```  

* ROS node (Python script)
  make sure your
  ```
  self.comm_freq = float( rospy.get_param('~odom_freq', '15') ) # hz of communication
  ```
  is higher than the rate the Arduino sends the message. For example:
  ```
  ***THIS IS GOOD***
  #define SERIAL_PERD 100000  // Serial tx/rx check period, 100 ms is 10 Hz
  ---------
  self.comm_freq = float( rospy.get_param('~odom_freq', '15') ) # hz of communication
  ```
  If you recieve less frequently than the transmit rate, packet will be lost and you'll potentially have a jumping odometry. Equal rate is not a good idea since simetimes there might be a slight difference between system clock, For example
  ```
  ***THIS IS BAD***
  #define SERIAL_PERD 100000  // Serial tx/rx check period, 100 ms is 10 Hz
  ---------
  self.comm_freq = float( rospy.get_param('~odom_freq', '10') ) # hz of communication
  ```

## Version
1.0
## Author
**Ewing Kang** - [EwingKang](https://github.com/EwingKang)
## License
GPLv3
## TODOs
[] github wiki
