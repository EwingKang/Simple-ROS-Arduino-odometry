#ifndef OdomMsg_h_
#define OdomMsg_h_
//Arduino Uno has a buffer size of 64-1 Bytes

typedef struct Odom_Msg{
	byte headerA, headerB;		//2B should be set to 0x00 0xFF 00000000 11111111
    unsigned int seq;			//2B, for sequencing
	unsigned long t_micro;		//4B, in arduino Micros() format
	float x, y, th;				//4B*3, state of the odom
	float v_x, v_y, omega;		//4B*3, state of the odom
};

typedef struct Enc_msg{
	byte headerA, headerB;		//2B should be set to 0x00 0xFE 00000000 11111110
	unsigned char seq;			//1B sequencing
	short dx, dy, dth;			//2B*3, in 0.1mm, maxed out at [+-3.2767] meters
};

typedef union Odom_Msg_Bfr {
   Odom_Msg odom_msg;
   byte msg[32];
};

typedef union Enc_Msg_Bfr {
   Enc_msg enc_msg;
   byte msg[9];
};
#endif