#include "r_axis_interrupt.h"
#include "general_defs.h"

volatile int r_num_of_steps = 0;
volatile bool r_dir;

void intermediator_setup_r(){

	cli();

	pinMode(RECIEVE_STEP_R, INPUT);
	pinMode(RECIEVE_DIR_R, INPUT);

	attachInterrupt(digitalPinToInterrupt(RECIEVE_DIR_R), dir_received_r, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP_R), step_received_r, CHANGE);

	if (digitalReadFast(RECIEVE_DIR_R) == 0){
		r_dir = RIGHT;
	} else {
		r_dir = LEFT;
	}

	sei();

	// delay(INTERMEDIATOR_DELAY);
}

void step_received_r() {
	r_num_of_steps += r_dir;

	#ifdef DEBUG_INTERRUPTS_R
		Serial.println(F("step_received_r"));
	#endif
}

void dir_received_r() {
	if (digitalReadFast(RECIEVE_DIR_R) == 0){
		r_dir = RIGHT;
	} else {
		r_dir = LEFT;
	}

	#ifdef DEBUG_INTERRUPTS_R
		Serial.println(F("dir_received_r"));
	#endif
}