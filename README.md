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
stuff
  
  
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
