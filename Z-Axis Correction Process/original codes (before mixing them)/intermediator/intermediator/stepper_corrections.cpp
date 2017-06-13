#include "stepper_corrections.h"

volatile byte state = ENABLED;

// long step_time_rise = 0;
// long last_time_difference = 0;
// long current_time_difference = 0;
// 	step_time_rise = micros();
// 	current_time_difference = micros() - step_time_rise;

void toggle_state(byte state){
	if (state == ENABLED){
		digitalWriteFast(PASS_ENABLE, ENABLED);
	} else {
		digitalWriteFast(PASS_ENABLE, DISABLED);
	}
}

void toggle(){
	#ifdef DEBUG_INTERRUPTS
		Serial.println("in toggle");
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");  
	#endif

	if (digitalReadFast(RECIEVE_ENABLE) == HIGH) {
		state = DISABLED;
		toggle_state(DISABLED);
	} else {
		state = ENABLED;
		toggle_state(ENABLED);
	}	  	
}

void direction_received() {
	digitalWriteFast(PASS_DIR, digitalReadFast(RECIEVE_DIR));
}

	// void step_received(){ // it's marlin's turn
	// 	#ifdef DEBUG_INTERRUPTS
	// 		Serial.println("in step_recieved");
	// 		Serial.print("state is: ");
	// 		Serial.println(state ? "DISABLED": "ENABLED");
	// 	#endif

	// 	current_time_difference = micros() - last_step_time;

	// 	last_step_time

	// 	if (state == ENABLED){
	// 		digitalWriteFast(PASS_STEP, LOW);
	// 		digitalWriteFast(PASS_STEP, HIGH);
	// 	}
	// }

void step_received_rising(){
	#ifdef DEBUG_INTERRUPTS
		Serial.println("in step_received_rising");
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");
	#endif

	if (state == ENABLED){
		digitalWriteFast(PASS_STEP, LOW);
		digitalWriteFast(PASS_STEP, HIGH);
	}
}

void step_received_falling(){
	#ifdef DEBUG_INTERRUPTS
		Serial.println("in step_received_falling");
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");
	#endif

	if (state == ENABLED){
		digitalWriteFast(PASS_STEP, HIGH);
		digitalWriteFast(PASS_STEP, LOW);
	}
}