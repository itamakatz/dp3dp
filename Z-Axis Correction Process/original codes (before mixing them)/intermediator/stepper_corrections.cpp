#include "stepper_corrections.h"

volatile byte state = ENABLED;

void toggle_state(byte state){
	if (state == ENABLED){
		digitalWriteFast(PASS_ENABLE, ENABLED);
	} else {
		digitalWriteFast(PASS_ENABLE, DISABLED);
	}
}

void toggle(){
	if (digitalReadFast(RECIEVE_ENABLE) == HIGH) {	
		state = DISABLED;
		toggle_state(DISABLED);
	} else {
		state = ENABLED;
		toggle_state(ENABLED);
	}		

	#ifdef DEBUG_INTERRUPTS
		Serial.println("in toggle");
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");
	#endif
}

void direction_received() {
	digitalWriteFast(PASS_DIR, digitalReadFast(RECIEVE_DIR));
}

void step_received() {
	digitalWriteFast(PASS_STEP, digitalReadFast(RECIEVE_STEP));
}