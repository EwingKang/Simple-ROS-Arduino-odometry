
// for PinChangeInt library
#define NO_PORTB_PIN_CHANGES    //digital pin8 to 13
#define NO_PORTC_PIN_CHANGES    //analog input (https://www.arduino.cc/en/Reference/PortManipulation)
#include "PinChangeInt/PinChangeInt.h"    // http://playground.arduino.cc/Main/PinChangeInt

// Ewing Car library
#define ENCODER_LOG_SIZE 20
#include "Wheel.h"
#include "Car.h"
#include "Odom.h"

// Vehicle settings
#define WHEEL_RAD 0.033f		// wheel radius in m
#define WHEEL_TOOTH 25		// tooth count
#define MAX_WHEEL_RPS 2		// maximum wheel revolution per second
#define B 0.1f			// b = wheel distance

// Main loop settings
#define PRINT_PERD 750000       // 750 ms is 4/3 Hz
#define SERIAL_PERD 100000     // 100 ms is 10 Hz
#define CAR_PERD 20000			// 20 ms is 50 Hz
const PROGMEM float expected_log = (float)MAX_WHEEL_RPS * (float)WHEEL_TOOTH / (1000000 / CAR_PERD);

Car duckiebot(6, 7, 4, 5, WHEEL_RAD, WHEEL_TOOTH, B);
Odom_msg odom;
Msg_block *msg_block = (Msg_block *)&odom;

unsigned long temp;
int serial_cmd;
bool start;
uint8_t i=0;

//info
const PROGMEM char end_info[] = "Serial ended:";

void decode_cmd(int &, bool &);
void enable_msg(Odom_msg&);

//void messageCb(const geometry_msgs::Vector3&);

void setup() {
	memset(&odom, 0, sizeof(odom));
	start = false;
	
	Serial.begin(57600);

	serial_cmd = Serial.read();		// -1 if not available
	
	while(serial_cmd != 'B') { 
		delay(500);
		Serial.print("waiting ");
		Serial.println(i);
		i++;
		serial_cmd = Serial.read();
	}
	start = true;
	enable_msg(odom);
	
	PCintPort::attachInterrupt(duckiebot.wheel_L.pinA, risingLA, RISING);
	PCintPort::attachInterrupt(duckiebot.wheel_L.pinB, risingLB, RISING);
	PCintPort::attachInterrupt(duckiebot.wheel_R.pinA, risingRA, RISING);
	PCintPort::attachInterrupt(duckiebot.wheel_R.pinB, risingRB, RISING);
}


void loop() {
	static unsigned long last_pub, last_update;
	unsigned long time_now = micros();
	
	if(start) {
		if ( (time_now - last_update) >= CAR_PERD) {
			if( !duckiebot.update(time_now) ){
				Serial.println("update err");
			}
			last_update = time_now;
		}
	 
		if( (time_now - last_pub) >= SERIAL_PERD ) {
			//odometry
			odom.t_micro = time_now;		//seconds
			odom.seq++;
			duckiebot.GetPose(odom.x, odom.y, odom.th);
			duckiebot.GetTwist(odom.v_x, odom.v_y, odom.omega);
			
			//sent data
			/*Serial.print("x:");
			Serial.print(odom.x);
			Serial.print("y:");
			Serial.print(odom.y);
			Serial.print("th:");
			Serial.println(odom.omega);*/
			Serial.write((uint8_t*)msg_block, sizeof(*msg_block));
			
			serial_cmd = Serial.read();
			decode_cmd(serial_cmd, start);
			last_pub = time_now;
		}
	}else {
		if( (time_now - last_pub) >= SERIAL_PERD ) {
			Serial.print("Paused:");
			Serial.println(i);
			i++;
			last_pub = time_now;
		}
		serial_cmd = Serial.read();
		decode_cmd(serial_cmd, start);
	}

}


void decode_cmd(int &cmd, bool &start) {
	if(cmd == -1) return;		//no input, do nothing
	
	if(cmd == 'B') {
		if(start) {
			memset(&odom, 0, sizeof(odom));
			start = false;
			Serial.println(end_info);
			i = 0;
		}else {
			start = true;
			enable_msg(odom);
		}
	}else if(cmd == 'R') {
		duckiebot.reset();
	}else{
		duckiebot.set_direction(cmd);
	}

}

void enable_msg(Odom_msg& odom) {
	memset(&(odom.flagA), 0, sizeof(odom.flagA));
	memset(&(odom.flagB), -1, sizeof(odom.flagB));
	return;
}


/* This is super ugly, unfortunately, OO is not an option with PinChangeInt lib*/
void risingLA(void) {
	temp = micros();
	duckiebot.wheel_L.risingA( temp );
	PCintPort::detachInterrupt(duckiebot.wheel_L.pinA);
	PCintPort::attachInterrupt(duckiebot.wheel_L.pinA, fallingLA, FALLING); //attach the falling end
}
void fallingLA(void) {
	temp = micros();
	duckiebot.wheel_L.fallingA( temp );
	PCintPort::detachInterrupt(duckiebot.wheel_L.pinA);
	PCintPort::attachInterrupt(duckiebot.wheel_L.pinA, risingLA, RISING); //attach the falling end
}
void risingLB(void) {
	temp = micros();
	duckiebot.wheel_L.risingB( temp );
	PCintPort::detachInterrupt(duckiebot.wheel_L.pinB);
	PCintPort::attachInterrupt(duckiebot.wheel_L.pinB, fallingLB, FALLING); //attach the falling end
}
void fallingLB(void) {
	temp = micros();
	duckiebot.wheel_L.fallingB( temp );
	PCintPort::detachInterrupt(duckiebot.wheel_L.pinB);
	PCintPort::attachInterrupt(duckiebot.wheel_L.pinB, risingLB, RISING); //attach the falling end
}
void risingRA(void) {
	temp = micros();
	duckiebot.wheel_R.risingA( temp );
	PCintPort::detachInterrupt(duckiebot.wheel_R.pinA);
	PCintPort::attachInterrupt(duckiebot.wheel_R.pinA, fallingRA, FALLING); //attach the falling end
}
void fallingRA(void) {
	duckiebot.wheel_R.fallingA( temp );
	PCintPort::detachInterrupt(duckiebot.wheel_R.pinA);
	PCintPort::attachInterrupt(duckiebot.wheel_R.pinA, risingRA, RISING); //attach the falling end
}
void risingRB(void) {
	temp = micros();
	duckiebot.wheel_R.risingB( temp );
	PCintPort::detachInterrupt(duckiebot.wheel_R.pinB);
	PCintPort::attachInterrupt(duckiebot.wheel_R.pinB, fallingRB, FALLING); //attach the falling end
}
void fallingRB(void) {
	temp = micros();
	duckiebot.wheel_R.fallingB( temp );
	PCintPort::detachInterrupt(duckiebot.wheel_R.pinB);
	PCintPort::attachInterrupt(duckiebot.wheel_R.pinB, risingRB, RISING); //attach the falling end
}