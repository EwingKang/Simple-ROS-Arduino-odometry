#ifndef Odom_h_
#define Odom_h_
//Arduino Uno has a buffer size of 64 Bytes

typedef struct Odom_msg{
	byte flagA, flagB;			//2B should be set to 00000000 11111111
    unsigned int seq;			//2B, for sequencing
	unsigned long t_micro;		//4B, in arduino Micros() format
	float x, y, th;				//4B*3, state of the odom
	float v_x, v_y, omega;		//4B*3, state of the odom
};

typedef union Msg_block {
   Odom_msg odom_msg;
   byte msg[32];
};
#endif