#ifndef Wheel_h_
#define Wheel_h_

#include<string.h>
//#include<math.h>

#ifndef ENCODER_LOG_SIZE
#define ENCODER_LOG_SIZE 10
#endif

class Wheel
{
public:
	uint8_t pinA, pinB;
	int8_t direction;
	long int posA, posB, last_posA, last_posB;	// global tooth counter
	float radius;
	uint8_t tooth;
	
	Wheel(uint8_t pin1, uint8_t pin2, float rad, uint8_t too) {
		this->pinA = pin1;
		this->pinB = pin2;
		radius = rad;
		tooth = too;

		pinMode(pinA, INPUT);     //set the pin_rc to input
		pinMode(pinB, INPUT);     //set the pin_rc to input
		
		reset();
	}
	
	void risingA(const unsigned long &time) {
		updatenow();
		riseA_t[now_i] = time;
		if (direction == 1)	posA++;
		else if (direction == -1)	posA--;
	}
	
	void fallingA(const unsigned long &time) {
		fallA_t[now_i] = time;
		
	}
	
	void risingB(const unsigned long &time) {
		return;
	}
	
	void fallingB(const unsigned long &time) {
		return;
	}
	
	void reset() {
		posA = 0;
		posB = 0;
		last_posA = 0;
		last_posB = 0;
		direction = 0;
		memset(riseA_t, 0, sizeof(riseA_t)); 
		memset(fallA_t, 0, sizeof(fallA_t)); 
		now_i = 0;
		last_i = 0;
		last_visit = 0;
	}
	

	float rotation_count() {
		return (float) posA / tooth;
	}
	
	// return angle in radius
	float angle() {
		return get_angle();
	}
	
	float translation() {
		return get_angle() * radius;
	}
	
	bool delta_D(unsigned long &duration, float &distance){
		uint8_t snap_now = now_i;
		int rise_count = posA - last_posA;
		if( riseA_t[snap_now] < riseA_t[last_visit] ) {
			// something is wrong, time should not go back
			distance = 0;
			duration = 0;
			last_visit = snap_now;
			last_posA += rise_count;
			return false;
		}
		distance = radius * 2*M_PI * ((float)rise_count / (float)tooth); // S = r*theta
		duration = riseA_t[snap_now] - riseA_t[last_visit];
		last_visit = snap_now;
		last_posA += rise_count;
		return true;
	}
	

	/////// differential group/////
	float angular_vel() {
		//TODO: filter maybe?
		uint8_t snap_now = now_i;
		uint8_t snap_last = last_i;
		return (2 * M_PI / tooth) / ( riseA_t[snap_now] - riseA_t[snap_last] );
	}
	
	float linear_vel() {
		return angular_vel() * radius;
	}

	
private:
	unsigned long riseA_t[ENCODER_LOG_SIZE], fallA_t[ENCODER_LOG_SIZE];
	uint8_t now_i, last_i;
	uint8_t last_visit;

	
	void updatenow() {
		now_i++;
		this->last_i = this->now_i - 1;
		now_i %= ENCODER_LOG_SIZE;
		return;
	}
	
	float get_angle() {
		return 2 * M_PI * (float) posA / tooth;
	}
};

#endif