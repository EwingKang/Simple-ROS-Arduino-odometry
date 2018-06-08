/****************************************
* ENC_TYPE options:
*	1	rising edge counter, need direction input
*	2	tachometer with rising and falling edge, need direction input 
*	4 	quadrature encder, auto direction
****************************************/
#define ENC_TYPE 4
#define ENC_ENABLE_CUMU		// enable cumulative counter

//========== VEHICLE CONFIG ==========//
#define WHEEL_RAD 0.033f		// wheel radius in m
#define B 0.1f					// b = wheel separation distance
#define ENC_REDUCTION 21		// encoder reduction ratio = (wheel rpm)/(encoder rpm)
#define MAX_WHEEL_RPS ((float)201/(float)60)	// maximum wheel revolution per second

//========== ENCODER CONFIG ==========//
#define ENC_TPR 334				// encoder tooth count
#define WHEEL_PPR (ENC_TYPE*ENC_TPR*ENC_REDUCTION)	// pulse count per wheel revolution
