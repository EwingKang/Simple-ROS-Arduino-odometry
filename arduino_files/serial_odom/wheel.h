#ifndef Wheel_h_
#define Wheel_h_

#import <Arduino.h> 	// because of order the compiler auto builds

#include<string.h>
#include<stdint.h>		// because of order the compiler auto builds
#include<math.h>

#define DIR_FORD 1
#define DIR_BACK -1
#define DIR_STOP 0

class Wheel
{
  public:
//=======================================================//
	const uint8_t pinA, pinB;
	
	/////// Constructor ///////
	#if ENC_TYPE == 1 || ENC_TYPE == 2
		Wheel(uint8_t pin, float rad, unsigned int too);
	#elif ENC_TYPE == 4
		Wheel(uint8_t pin1, uint8_t pin2, float rad, unsigned int too);
	#else
		#error error Encoder type incorrect or not set
	#endif
	
	/////// callback group ///////
	void Cb_RiseA(void);
	void Cb_RiseB(void);
	void Cb_FallA(void);
	void Cb_FallB(void);

	/////// state machine group ///////
	void update(const unsigned long &t_now);		// update calculation
	void fast_update(int16_t &enc);			// fast update. should not be used with update() simultaneously
	void reset();				// reset counter
	void set_dir(int8_t dir) {direction = dir;}
	
	/////// calculation group ///////
	//return the value calculated at last update
	int get_count() { return cntr; }
	float get_rev() { return (float)cntr / ppr; }
	float get_dth() { return 2*M_PI * get_rev(); }  	// return the forward distance at last update
	float get_dx()  { return get_dth()*radius; } 	// return the forward distance at last update
	float get_angular_vel()	{return get_dth() * 1e6 /duration_us; }
	float get_linear_vel() 	{return get_angular_vel() * radius;	}
	
	/////// statictics group ///////
	#ifdef ENC_ENABLE_CUMU
	long cumulative_cntr() 		{return cntr_cumu; }
	float cumulative_revo()		{return (float)cntr_cumu / ppr; }		// revolution count
	float cumulative_angle() 	{return 2*M_PI * cumulative_revo(); } // in radians
	float cumulative_dist() 	{return cumulative_angle() * radius; }
	
	unsigned long total_cntr() 	{return cntr_total;}
	float total_revo()			{return (float)cntr_total/ppr;}
	float total_angle() 		{return 2*M_PI * total_revo(); }// abs cumulative value
	float total_dist()			{return total_angle() * radius;}// absolute cumulative value
	#endif

//=======================================================//
  private:
	const float radius;
	const uint16_t ppr;	// pulse per wheel revolution for one phase

	int8_t pin_state;
	int8_t direction;
	int16_t cntr, int_cntr;	// output counter and interrupt counter
	#ifdef ENC_ENABLE_CUMU
	int32_t cntr_cumu; 		// cumulative counter, updated with update() call
	uint32_t cntr_total;	// abs counter
	#endif
	
	unsigned long last_call_t_us;	// last update time
	unsigned long duration_us; 		// duration from last to 
};

#endif