//
// Created by liav on 29/05/17.
//

#include "stepper_corrections.h"

int g_steps_gained_from_marlin = 0;


FORCE_INLINE void _enable() {
	digitalWriteFast(PASS_ENABLE, ENABLED);
}

FORCE_INLINE void _disable() {
	digitalWriteFast(PASS_ENABLE, DISABLED);
}


void toggle_state(byte state){
	if (state == ENABLED){
		_enable();
	} else {
		_disable();
	}
}


void apply_steps(int num_of_steps) {
	#ifdef MULTIPLE_STEPS
		REPEAT(num_of_steps){
	#endif
			digitalWriteFast(PASS_STEP, LOW);
			digitalWriteFast(PASS_STEP, HIGH);
	#ifdef MULTIPLE_STEPS
		}
	#endif
	g_steps_gained_from_marlin += num_of_steps;
}


void set_direction() {
	digitalWriteFast(PASS_DIR, digitalReadFast(RECIEVE_DIR));
}


void toggle(){
	#ifdef DEBUG_INTERRUPTS
		Serial.println("in toggle");
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");  
	#else
		if (digitalReadFast(RECIEVE_ENABLE) == HIGH){
			state = DISABLED;
			toggle_state(DISABLED);
		} else {
			state = ENABLED;
			toggle_state(ENABLED);
		}	  	
	#endif
}


void step_received(){ // it's marlin's turn
	#ifdef DEBUG_INTERRUPTS
		Serial.println("in step_recieved");
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");
	#else
		if (state == ENABLED){
		  	apply_steps(1);
		}
	#endif
}