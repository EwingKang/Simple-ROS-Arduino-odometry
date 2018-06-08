#include "vehicle_config.h"
#if ENC_TYPE == 4
#include "wheel.h"

#define ENC_ST_H 0xFF	//11111111
#define ENC_ST_L 0x00	//00000000

#define ENC_A 0x01		//00000001
#define ENC_B 0x02		//00000010

/***************************************
 constructor
****************************************/
Wheel::Wheel(uint8_t pin1, uint8_t pin2, float rad, unsigned int too):
								pinA(pin1), pinB(pin2), radius(rad), ppr(too)
{
	pinMode(pinA, INPUT);     //set the pin_rc to input
	pinMode(pinB, INPUT);     //set the pin_rc to input
	reset();
}

/***************************************
 pin interrupt callbacks
****************************************/
void Wheel::Cb_RiseA(void) {
	if( pin_state&ENC_A ) return;		// immediately return if A is already high
	if( pin_state&ENC_B ) int_cntr--;	// if pinB is HIGH
	else                  int_cntr++;
	pin_state |= ENC_A;			// set pinA High
	#ifdef ENC_ENABLE_CUMU
	  cntr_total++;
	#endif	
	return;
}
void Wheel::Cb_FallA(void) {
	if( !(pin_state&ENC_A) ) return;	// immediately return if A is already low
	if( pin_state&ENC_B ) int_cntr++;	// if pinB is HIGH
	else                  int_cntr--;
	pin_state &= (~ENC_A);		// set pinA low
	#ifdef ENC_ENABLE_CUMU
	  cntr_total++;
	#endif	
	return;
}
void Wheel::Cb_RiseB(void) {
	if( pin_state&ENC_B ) return;		// immediately return if B is already high
	if( pin_state&ENC_A ) int_cntr++;	// if pinA is HIGH
	else                  int_cntr--;
	pin_state |= ENC_B;		// set pinB High
	#ifdef ENC_ENABLE_CUMU
	  cntr_total++;
	#endif	
	return;
}
void Wheel::Cb_FallB(void) {
	if( ! (pin_state&ENC_B) ) return;	// immediately return if B is already low
	if( pin_state&ENC_A ) int_cntr--;	// if pinA is HIGH
	else                  int_cntr++;
	pin_state &= (~ENC_B);		// set pinA low
	#ifdef ENC_ENABLE_CUMU
	  cntr_total++;
	#endif	
	return;
}

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
	pin_state = 0;
	#ifdef ENC_ENABLE_CUMU
	  cntr_cumu = 0; 	// cumulative counter, updated with update() call
	  cntr_total = 0;	// abs counter
	#endif
}

#endif