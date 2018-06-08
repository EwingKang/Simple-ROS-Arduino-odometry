#ifndef Car_h_
#define Car_h_

#include<math.h>
#include "wheel.h"

class Car {
public:
	const float b; 	// B = wheel separation distance
	//float pos_cov[36], vel_cov[36];
	Wheel wheel_l, wheel_r;
	
	/**********************************************************
							Constructor
	***********************************************************/
	#if ENC_TYPE == 1 || ENC_TYPE == 2
	  Car(uint8_t pinL, uint8_t pinR, float wheel_rad, unsigned int wheel_tooth, float sep)
		:	wheel_l(pinL, wheel_rad, wheel_tooth)
		,	wheel_r(pinR, wheel_rad, wheel_tooth) 
		,	b(sep)
	  {
		last_t = micros();
	  }
	#elif ENC_TYPE == 4
	  Car(uint8_t pinLA, uint8_t pinLB, uint8_t pinRA, uint8_t pinRB, float wheel_rad, unsigned int wheel_tooth, float sep)
		:	wheel_l(pinLA, pinLB, wheel_rad, wheel_tooth)
		,	wheel_r(pinRA, pinRB, wheel_rad, wheel_tooth) 
		,	b(sep)
	  {
		last_t = micros();
	  }
	#else
		#error error Encoder type incorrect or not set
	#endif
	
	/**********************************************************
						Vehicl state update
		Func: update and calculate vehicle state according to 
		      encoder counters.
		Input: arduino time in microseconds
		Output: true/success false/error
	***********************************************************/
	bool update(const unsigned long &time_now) {
		unsigned long dt;
		float ds_l = 0, ds_r = 0;
		float dS, dth, dx, dy;
		
		if(paused) return false;
		
		wheel_l.update(time_now);
		wheel_r.update(time_now);
		ds_l = wheel_l.get_dx();
		ds_r = wheel_r.get_dx();
	
		dS = (ds_l + ds_r) / 2;
		dth = (ds_r - ds_l) / b;
		dx = dS * cos(theta + dth/2);
		dy = dS * sin(theta + dth/2);
		
		x += dx;
		y += dy;
		theta += dth;
		
		dt = time_now - last_t;
		
		if (dt == 0){
			v_x = 0;
			v_y = 0;
			omega = 0;
		}else{
			v_x = dS / dt;
			v_y = 0;
			omega = dth / dt; 
		}
		last_t = time_now;
		return true;
	}
	
	/**********************************************************
						Vehicl fast update
		Func: update value without expansive calculations
		Input: arduino time in microseconds
		Output: true/success false/error
		Note: *** THERE SHOULD BE ONLY ONE CALL TO EITHER ***
		      *** update() OR fast_update()               ***
	***********************************************************/
	void fast_update(short &enc_l, short &enc_r) {
		wheel_l.fast_update((int16_t &)enc_l);
		wheel_r.fast_update((int16_t &)enc_r);
		return;
	}
	
	/**********************************************************
						Set Vehicle direction
		Func: Set direction of the wheels according to
		      directional information. Using WASD convention.
		Input: ascii char from serial
		Output:
		Note: ONLY USEFULL WHEN NOT USING QUADRATURE ENCODERS
	***********************************************************/
	#if ENC_TYPE == 1 || ENC_TYPE == 2
	  void set_direction(const char &cmd){
		if(cmd == 'W' || cmd == 'E' || cmd == 'D') wheel_l.set_dir(DIR_FORD);
		else if (cmd == 'S' || cmd == 'A' || cmd == 'Z') wheel_l.set_dir(DIR_BACK);
		else if (cmd == 'X' || cmd == 'Q' || cmd == 'C') wheel_l.set_dir(DIR_STOP);
		
		if(cmd == 'W' || cmd == 'Q' || cmd == 'A') wheel_r.set_dir(DIR_FORD);
		else if (cmd == 'S' || cmd == 'D' || cmd == 'C') wheel_r.set_dir(DIR_BACK);
		else if (cmd == 'X' || cmd == 'E' || cmd == 'Z') wheel_r.set_dir(DIR_STOP);
		
		return;
	  }
	#endif
	
	/**********************************************************
							System pause
		Func: pause system TODO and all interrupts
		Input: true/pause false/unpause
		Output:
	***********************************************************/
	void pause(const bool is_paused){
		this->paused = is_paused;
		#if ENC_TYPE == 1 || ENC_TYPE == 2
		if(is_paused) {
			wheel_l.set_dir(0);
			wheel_r.set_dir(0);
		}
		#endif
			
	}
	
	/**********************************************************
							System reset
		Func: reset all value
		Input:
		Output:
	***********************************************************/
	void reset(){
		wheel_l.reset();
		wheel_r.reset();
		x = 0;
		y = 0;
		theta = 0;
		v_x = 0;
		v_y = 0;
		omega = 0;
		last_t = micros();
		return;
	}
	
	/**********************************************************
						Get vehicle state
		Func:
		Input: reference to state
		Output: (by reference) x(m), y(m), theta(radians)
	***********************************************************/
	void GetPose(float &x, float &y, float &th) {
		x = this -> x;
		y = this -> y;
		th = this -> theta;
		return;
	}
	
	/**********************************************************
					Get vehicle state derivatives
		Func:
		Input: reference to state derivatives
		Output: (by reference) x(m/s), y(m/s), theta(radians)
	***********************************************************/
	void GetTwist(float &vx, float &vy, float &omega) {
		vx = this -> v_x;
		vy = this -> v_y;
		omega = this -> omega;
		return;
	}
	
	
private:
	float x, y, theta;
	float v_x, v_y, omega;
	bool paused;
	unsigned long last_t;
};

#endif
