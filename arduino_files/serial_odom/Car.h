#ifndef Car_h_
#define Car_h_

#include<math.h>
#include "Wheel.h"

class Car {
public:
	float L; 	// 2L = wheel separation distance
	//float pos_cov[36], vel_cov[36];
	Wheel wheel_L, wheel_R;

	Car(uint8_t pinLA, uint8_t pinLB, uint8_t pinRA, uint8_t pinRB, float wheel_rad, uint8_t wheel_tooth, float l)
		:	wheel_L(pinLA, pinLB, wheel_rad, wheel_tooth)
		,	wheel_R(pinRA, pinRB, wheel_rad, wheel_tooth) 
	{
		this -> L = l;
	}
	
	bool update(const unsigned long &time_now) {
		unsigned long dt_L, dt_R, dt;
		float ds_L, ds_R;
		float dS, dth, dx, dy;
		
		dt_L = 0;
		dt_R = 0;
		ds_L = 0;
		ds_R = 0;
		if( !wheel_L.delta_D(dt_L, ds_L) || !wheel_R.delta_D(dt_R, ds_R) ) {
			// wheel error!
			Serial.println("Wheel err");
			return false;
		}
		
		dS = (ds_L + ds_R) / 2;
		dth = (ds_R - ds_L) / (2*L);
		dx = dS * cos(theta + dth/2);
		dy = dS * sin(theta + dth/2);
		
		x += dx;
		y += dy;
		theta += dth;
		
		dt = max(dt_L, dt_R);
		
		if (dt == 0){
			v_x = 0;
			v_y = 0;
			omega = 0;
		}else{
			v_x = dx / dt;
			v_y = dy / dt;
			omega = dth / dt; 
		}
		return true;
	}
	
	void GetPose(float &x, float &y, float &th) {
		x = this -> x;
		y = this -> y;
		th = this -> theta;
		return;
	}
	
	void GetTwist(float &vx, float &vy, float &omega) {
		vx = this -> v_x;
		vy = this -> v_y;
		omega = this -> omega;
		return;
	}
	
	void set_direction(const int &cmd){
		if(cmd=='X') {
			reset();
			return;
		}
		
		if(cmd == 'W' || cmd == 'E' || cmd == 'D') wheel_L.direction = 1;
		else if (cmd == 'S' || cmd == 'A' || cmd == 'Z') wheel_L.direction = -1;
		else if (cmd == ' ' || cmd == 'Q' || cmd == 'C') wheel_L.direction = 0;
		
		if(cmd == 'W' || cmd == 'Q' || cmd == 'A') wheel_R.direction = 1;
		else if (cmd == 'S' || cmd == 'D' || cmd == 'C') wheel_R.direction = -1;
		else if (cmd == ' ' || cmd == 'E' || cmd == 'Z') wheel_R.direction = 0;
		return;
	}
	
	void reset(){
		wheel_L.reset();
		wheel_R.reset();
		x = 0;
		y = 0;
		theta = 0;
		v_x = 0;
		v_y = 0;
		omega = 0;
		return;
	}
	
private:
	float x, y, theta;
	float v_x, v_y, omega;	
};

#endif