#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <math.h>

void tfPub1();
//void tfPub2();

ros::Publisher v1_pub, v2_pub, v3_pub;

int main(int argc, char **argv) {
   
	// ROS Init    
	ros::init(argc, argv, "tf_test");
	ros::NodeHandle nh;
	v1_pub = nh.advertise<geometry_msgs::Vector3Stamped>("v1", 10);
	v2_pub = nh.advertise<geometry_msgs::Vector3Stamped>("v2", 10);
	v3_pub = nh.advertise<geometry_msgs::Vector3Stamped>("v3", 10);
	ros::Rate loop_rate(10);
	
	ROS_INFO("TF test starting");
	while(ros::ok()) {
		tfPub1();
		//tfPub2();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	ROS_INFO("Shutting down");
	
}
void tfPub1(){
	tf::Quaternion q1, q2, q3;
	q1.setEuler(10*M_PI/180, 25*M_PI/180, 30*M_PI/180);
	q2.setEuler(-10*M_PI/180, -25*M_PI/180, -30*M_PI/180);
	q3.setEuler(-50*M_PI/180, 10*M_PI/180, 5*M_PI/180);
	
	static tf::TransformBroadcaster tfbr1, tfbr2, tfbr3, tfbr4, tfbr5, tfbr6, tfbr7;
	tf::Transform tf1(q1, tf::Vector3(1, 2, 0));
	tf::Transform tf2(q2, tf::Vector3(0, 0, 3));
	tf::Transform tf3 = tf1*tf2;
	tf::Transform tf4 = tf2*tf1;
	tf::Transform tf5_inW;
	tf5_inW.setOrigin( tf1.getBasis()*tf2.getOrigin() );
	tf5_inW.setRotation( tf1.getRotation() * tf2.getRotation() );
	tf::Transform tf6_inW = tf1 * tf2 * tf1.inverse();
	tf::Transform tf7_inW = tf1.inverse() * tf2 * tf1;

	ros::Time time = ros::Time::now();
	tfbr1.sendTransform(tf::StampedTransform(tf1, time, "world", "tf1"));
	tfbr2.sendTransform(tf::StampedTransform(tf2, time, "tf1", "tf2"));
	tfbr3.sendTransform(tf::StampedTransform(tf3, time, "world", "tf1xtf2"));
	tfbr4.sendTransform(tf::StampedTransform(tf4, time, "world", "tf2xtf1"));
	tfbr5.sendTransform(tf::StampedTransform(tf5_inW, time, "world", "tf5inW"));
	tfbr6.sendTransform(tf::StampedTransform(tf6_inW, time, "world", "tf1*tf2*tf1^-1"));
	tfbr7.sendTransform(tf::StampedTransform(tf7_inW, time, "world", "tf1^-1*tf2*tf1"));
	
}
/*
void tfPub2(){
	ros::Time time = ros::Time::now();
	static int i = 0;
	
	tf::Quaternion q1, q2, q3;
	
	q2.setEuler(-10*M_PI/180, -25*M_PI/180, -30*M_PI/180);
	q3.setEuler(-50*M_PI/180, 10*M_PI/180, 5*M_PI/180);
	
	static tf::TransformBroadcaster tfbr1, tfbr2;
	tf::Transform tf1(q1, tf::Vector3(-1, -2, 0));
	tf::Transform tf2(q2, tf::Vector3(0, 0, 2));
	
	geometry_msgs::poseStamped pose1, pose2, pose3, pose4;
	tf::Vector3 tf_pose1(1, 2, 3), tf_pose2(3, 2, 1);
	pose1.header.seq = i;
	pose1.header.stamp = time;
	pose1.header.frame_id = "tf1";
	tf::poseTFToMsg( tf_pose1, pose1.pose );
	
	pose2.header.seq = i;
	pose2.header.stamp = time;
	pose2.header.frame_id = "tf2";
	tf::poseTFToMsg( tf_pose2, pose2.pose );
	
	pose3.header.seq = i;
	pose3.header.stamp = time;
	pose3.header.frame_id = "tf1";
	tf::poseTFToMsg( (tf1.inverse()*tf2) * tf_pose2, pose2.pose );
	
	pose4.header.seq = i;
	pose4.header.stamp = time;
	pose4.header.frame_id = "tf1";
	//tf::vector3TFToMsg( (tf1.inverse()*tf2) * tf_vec2 * (tf1.inverse()*tf2).inverse(), 
						//vec4.vector);
	
	v1_pub.publish(vec1);
	v2_pub.publish(vec2);
	v3_pub.publish(vec3);
	//v4_pub.publish(vec4);

	tfbr1.sendTransform(tf::StampedTransform(tf1, time, "world", "tf_vec1"));
	tfbr2.sendTransform(tf::StampedTransform(tf2, time, "world", "tf_vec2"));
	
	i++;
	
}*/