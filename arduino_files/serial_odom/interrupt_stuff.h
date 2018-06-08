extern Car duckiebot;
/***************************************
 prototypes
***************************************/
void risingLA(void);
void fallingLA(void);
void risingLB(void);
void fallingLB(void);
void risingRA(void);
void fallingRA(void);
void risingRB(void);
void fallingRB(void);

/***************************************
 Interfaces
***************************************/
void enable_encoder_int_type1() {
	enableInterrupt(duckiebot.wheel_l.pinA, risingLA, RISING);
	enableInterrupt(duckiebot.wheel_r.pinA, risingRA, RISING);
}
void enable_encoder_int_type2() {
	enableInterrupt(duckiebot.wheel_l.pinA, risingLA, RISING);
	enableInterrupt(duckiebot.wheel_r.pinA, risingRA, RISING);
}
void enable_encoder_int_type4() {
	enableInterrupt(duckiebot.wheel_l.pinA, risingLA, RISING);
	enableInterrupt(duckiebot.wheel_r.pinA, risingRA, RISING);
	enableInterrupt(duckiebot.wheel_l.pinB, risingLB, RISING);
	enableInterrupt(duckiebot.wheel_r.pinB, risingRB, RISING);
}

void disable_encoder_int() {
	disableInterrupt(duckiebot.wheel_l.pinA);
	disableInterrupt(duckiebot.wheel_r.pinA);
	#if ENC_TYPE == 4
	disableInterrupt(duckiebot.wheel_l.pinB);
	disableInterrupt(duckiebot.wheel_l.pinB);
	#endif
}

//Note: interfacing functions are down below
/********************************************
global callbacks for interrupt library
This is super ugly, unfortunately, OO is not
an option with PinChangeInt lib
*********************************************/
void risingLA(void) {
	duckiebot.wheel_l.Cb_RiseA();
	#if ENC_TYPE != 1
	enableInterrupt(duckiebot.wheel_l.pinA, fallingLA, FALLING); //attach the falling end
	#endif
}
void fallingLA(void) {
	duckiebot.wheel_l.Cb_FallA();
	enableInterrupt(duckiebot.wheel_l.pinA, risingLA, RISING); 
}
void risingLB(void) {
	duckiebot.wheel_l.Cb_RiseB();
	enableInterrupt(duckiebot.wheel_l.pinB, fallingLB, FALLING); 
}
void fallingLB(void) {
	duckiebot.wheel_l.Cb_FallB();
	enableInterrupt(duckiebot.wheel_l.pinB, risingLB, RISING); 
}
void risingRA(void) {
	duckiebot.wheel_r.Cb_RiseA();
	#if ENC_TYPE != 1
	enableInterrupt(duckiebot.wheel_r.pinA, fallingRA, FALLING); 
	#endif
}
void fallingRA(void) {
	duckiebot.wheel_r.Cb_FallA();
	enableInterrupt(duckiebot.wheel_r.pinA, risingRA, RISING);
}
void risingRB(void) {
	duckiebot.wheel_r.Cb_RiseB();
	enableInterrupt(duckiebot.wheel_r.pinB, fallingRB, FALLING);
}
void fallingRB(void) {
	duckiebot.wheel_r.Cb_FallB();
	enableInterrupt(duckiebot.wheel_r.pinB, risingRB, RISING);
}
