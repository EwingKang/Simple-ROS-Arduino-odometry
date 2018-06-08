#include "vehicle_config.h"
#if ENC_TYPE == 2
#include "wheel.h"

#define ENC_ST_H 1
#define ENC_ST_L 0

/***************************************
 constructor
****************************************/
Wheel::Wheel(uint8_t pin, float rad, unsigned int too): 
								pinA(pin), pinB(0), radius(rad), ppr(too)
{
	pinMode(pinA, INPUT);     //set the pin_rc to input
	reset();
}

/***************************************
 pin interrupt callbacks
****************************************/
void Wheel::Cb_RiseA(void) {
	if( pin_state == ENC_ST_H) return;			// immediately return if overlaping state
	if (direction == DIR_FORD)		int_cntr++;
	else if (direction == DIR_BACK)	int_cntr--;
	pin_state = ENC_ST_H;
	#ifdef ENC_ENABLE_CUMU
	  cntr_total++;
	#endif	
	return;
}
void Wheel::Cb_FallA(void) {
	if( pin_state == ENC_ST_L) return;			// immediately return if overlaping state
	if (direction == DIR_FORD)		int_cntr++;
	else if (direction == DIR_BACK)	int_cntr--;
	pin_state = ENC_ST_L;
	#ifdef ENC_ENABLE_CUMU
	  cntr_total++;
	#endif	
	return;
}
//void Wheel::Cb_RiseB(const unsigned long &time) {return;}
//void Wheel::Cb_FallB(const unsigned long &time) {return;}


/***************************************
 update calculation
****************************************/
void Wheel::update(const unsigned long &t_now){
	cntr = int_cntr;		// copy interrupt counts
	#ifdef ENC_ENABLE_CUMU
	  cntr_cumu += int_cntr;
	#endif	
	int_cntr = 0;			// reset interrupt counts
	duration_us = t_now - last_call_t_us;
	last_call_t_us = t_now;
}
/*******************************************************************
			fast update 
	Func: No duration or velocity calculations, useful for super 
          fast encoders.
	Input: Reference to encoder counter
	Output: interrupt counts
	Note: *** THERE SHOULD BE ONLY ONE CALL TO EITHER update() OR
              fast_update() ***
*******************************************************************/
void Wheel::fast_update(int16_t &enc){
	enc = int_cntr;		// copy interrupt counts
	int_cntr = 0;			// reset interrupt counts
	return;
}

/***************************************
 reset counter
****************************************/
void Wheel::reset(){
	cntr = 0;
	int_cntr = 0;	// output counter and interrupt counter
	direction = 0;
	pin_state = 3;
	#ifdef ENC_ENABLE_CUMU
	  cntr_cumu = 0; 	// cumulative counter, updated with update() call
	  cntr_total = 0;	// abs counter
	#endif
}

#endif