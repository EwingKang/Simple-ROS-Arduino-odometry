# Simple-ROS-Arduino-odometry
I started this project because I'm done with rosserial: it is just too bulky for the arduino's 2KB flash, and almost impossible to debug. The only two things it does is refuse to connect or report trying to sync. Trust me, it's much quicker to just write your own serial communication. It's quicker even I've never written any Python and actually learning Python at the same time!  
    For more information, please see [About the Code](#project-details) section below.
    
**PLEASE CHECK [ENCODER](#encoder) SECTION BELOW BEFORE PROCEEDING**
    
  
## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites
Tested and built with ROS-kinetic with Ubuntu 16.04LTS, but should work with any version near enough. 
Also I have Python 2.7. 
Please make sure you have the following items setup or at least you know what you're doing: 
* [ROS installed](http://wiki.ros.org/kinetic/Installation/Ubuntu) (tested in ROS-kinetic) 
* [have a ROS catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) (it is recommanded to put your source into .bashrc) 
* [Install the Arduino IDE](https://www.arduino.cc/en/Guide/Linux) ( I recommand to put the Arduino IDE under /opt/ before running install.sh ) 
* You'll need some way to publish your wheel command on topic "encoder" with type geometry_msgs.msg::Vector3. vec.x() is the left wheel, while vec.y() is the right. Grater than zero ( > 0) means forward, (<0) backward, (=0) means stop.

### Costumization  
To make sure you have the best result, it is necessary to make sure you have your parameters set-up, both in the script and on the Arduino.

* For Arduino program  
    A. Physical parameters:
    ```
    #define WHEEL_RAD 0.033f	// wheel radius in m
    #define WHEEL_TOOTH 25		// tooth count
    #define B 0.1f		        // b = distance between wheels
    ```  
    B. Loop system  
    ```
    #define SERIAL_PERD 100000  // Serial tx/rx check period, 100 ms is 10 Hz
    #define CAR_PERD 20000	    // Main calculation period, 20 ms is 50 Hz
    ```  
    C. Encoder interrupt memory (important)
    You have to make sure your expected_log < ENCODER_LOG_SIZ is true. For simplicity, the program will not raise any warning if you have any incorrect settings. Detailed explaination is in the Arduino section [below](#arduino).
    ```
    #define ENCODER_LOG_SIZE 20
    ...
    #define MAX_WHEEL_RPS 2		// maximum wheel revolution per second
    expected_log = (float)MAX_WHEEL_RPS * (float)WHEEL_TOOTH / (1000000 / CAR_PERD);
    ```  
* For python script
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
### Installation

1. Go to your catkin directory, clone the project into folder /serial_odom.
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/EwingKang/Simple-ROS-Arduino-odometry serial_odom
   ```
2. Link(or move) the Ardunio program to workspace. It is possible to open the (project)/arduino_files/serial_odom/serial_odom.ino with Arduino IDE directly. But I find it faster to find if everything is under Arduino IDE directory. You'll see a link folder in the Arduino/ afterwards
   ```
   ln -s ~/catkin_ws/src/serial_odom/arduino_files/serial_odom/ ~/Arduino/
   ```
3. "Compile" the file with catkin_make so ros recgonize the package
   ```
   cd ~/catkin_ws/
   catkin_make
   ```
4. Upload the Arduino code with Arduino IDE

### Run the node
Hook up all your cable, make sure you have the correct device (in this case /dev/ttyACM0), and simply rosrun:
```
rosrun serial_odom serial_odom.py _port:=/dev/ttyACM0
```
The code will reset the Arduino automatically, and you'll see:
1. Arduino waiting for communication
2. Arduino start sending odometry packet, serial is syncing 
3. Serial is synced and first decodable packet will have a mismatched sequence number.
4. Now the serial odometry is working.
  
## Project details  

### Encoder
**IMPORTANT: This project is bulid around a single phase encoder**  
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
On the Arduino side, data is struted and can be easily modified. 0x00FF is used as the starting signal of each packet.
 
### Line-by-line explaination
see wiki  


  
## Authors

* **Ewing Kang** - [EwingKang](https://github.com/EwingKang)

## License

## TODO
