
//========== LIBRARIES ==========//
#include "EnableInterrupt/EnableInterrupt.h" 	//Software interrupt lib
												//https://github.com/GreyGnome/EnableInterrupt
												//PinChangeInt is deprecated

#include "vehicle_config.h"	// USER customize settings
#include "car.h"			// Car library
#include "uart_odom_msg.h"	// UART message

//========== DEBUG ==========//
//#define DEBUG_SER_MON		// serial debug output
const PROGMEM float expected_int_rate = WHEEL_PPR * MAX_WHEEL_RPS;

//========== STATE/COMM CONFIG ==========//
#define SERIAL_PERD 20000		// 20  ms is  50 hz
#define CAR_PERD 10000			// 10  ms is 100 hz
#define PAUSED_PERD 500000		// 500 ms is   2 hz

//========== OBJECTS ==========//
Car duckiebot(4, 5, 6, 7, WHEEL_RAD, WHEEL_PPR, B);		// general quadrature encoder
//Car duckiebot(4, 5, WHEEL_RAD, WHEEL_PPR, B);			// only two pins if ENC_TYPE is 1 or 2
Odom_Msg odom_msg;
Odom_Msg_Bfr *odom_msg_ptr = (Odom_Msg_Bfr *)&odom_msg;	//byte block of Odom_msg

bool loop_on;
uint8_t seq;

// prototypes
void decode_cmd(int &, bool &);
void set_msg_header(Odom_Msg&);

#include "interrupt_stuff.h"

void setup() {
	// clear all
	memset(&odom_msg, 0, sizeof(odom_msg));
	loop_on = false;
	seq = 0;
	Serial.begin(115200);

	// hold here until starting signal
	while(Serial.read() != 'B') { 
		delay(500);
		#ifdef DEBUG_SER_MON
			Serial.println("***Debug Mode, full text output***");
		#endif
		Serial.print("Waiting: ");
		Serial.print(seq);
		Serial.println(", \"B\" to start");
		seq++;
	}
	
	// initialize object
	loop_on = true;
	seq = 0;
	set_msg_header(odom_msg);
	duckiebot.reset();
	while(Serial.available()) { Serial.read(); }	// flush input buffer
	
	#if ENC_TYPE == 1
	  enable_encoder_int_type1();
	#elif ENC_TYPE == 2
	  enable_encoder_int_type2();
	#elif ENC_TYPE == 4
	  enable_encoder_int_type4();
	#else 
	  #error incorrect encoder settings
	#endif
}

void loop() {
	static unsigned long last_pub, last_update;		// time keeper
	unsigned long time_now = micros();
	
	if(loop_on) {
		if ( (time_now - last_update) >= CAR_PERD) {
			if( !duckiebot.update(time_now) ){		// update car
				Serial.println("update err");
			}
			last_update = time_now;
		}
	 
		if( (time_now - last_pub) >= SERIAL_PERD ) {
			// odometry message update
			odom_msg.t_micro = time_now;			//micro seconds
			odom_msg.seq++;
			duckiebot.GetPose(odom_msg.x, odom_msg.y, odom_msg.th);			//pose: coordinate
			duckiebot.GetTwist(odom_msg.v_x, odom_msg.v_y, odom_msg.omega);	//velocity in body frame
			
			//sent data
			#ifdef DEBUG_SER_MON
				//Serial.print("x:");
				//Serial.print(odom_msg.x);
				//Serial.print("y:");
				//Serial.print(odom_msg.y);
				//Serial.print("th:");
				//Serial.println(odom_msg.omega);
				Serial.print("A dt:");
				Serial.print(duckiebot.wheel_l.get_count());
				Serial.print("c:");
				Serial.print(duckiebot.wheel_l.cumulative_cntr());
				Serial.print("R:");
				Serial.println(duckiebot.wheel_l.cumulative_revo());
			#else
				Serial.write((uint8_t*)odom_msg_ptr, sizeof(*odom_msg_ptr));
			#endif
			
			//input parsing
			decode_cmd(Serial.read(), loop_on);
			last_pub = time_now;
		}
	}else {		//if (loop_on)
		if( (time_now - last_pub) >= PAUSED_PERD ) {
			Serial.print("Paused:");
			Serial.println(seq);
			seq++;
			last_pub = time_now;
		}
		decode_cmd(Serial.read(), loop_on);		// continuous cmd parsing if not looping
	}

}

/*******************************************************************
			decode command
	Func:
	Input: 
	Output:
	Note: 
*******************************************************************/
void decode_cmd(int cmd, bool &loop_on) {
	if(cmd == -1) return;		///no input, do nothing
	
	if(cmd == 'P') {			/// system toggle pause
		if(loop_on) {				// pause
			loop_on = false;
			duckiebot.pause(true);
		}else {						// start
			loop_on = true;
			set_msg_header(odom_msg);
			duckiebot.pause(false);
		}
	}else if(cmd == 'R') {		/// system reset
		loop_on = false;
		duckiebot.reset();
		memset(&odom_msg, 0, sizeof(odom_msg));
		seq = 0;
		set_msg_header(odom_msg);
	}else if(cmd == 'B') {		/// system start
		loop_on = true;
		set_msg_header(odom_msg);
		duckiebot.pause(false);
	}else{						/// decode direction command
		#if ENC_TYPE == 1 || ENC_TYPE == 2
		  duckiebot.set_direction(cmd);
		#endif
	}
	return;
}

/*******************************************************************
			set message header
	Func:
	Input: 
	Output:
	Note: 
*******************************************************************/
void set_msg_header(Odom_Msg& odom_msg) {
	memset(&(odom_msg.headerA), 0x00, sizeof(odom_msg.headerA));
	memset(&(odom_msg.headerB), 0xFF, sizeof(odom_msg.headerB));
	return;
}