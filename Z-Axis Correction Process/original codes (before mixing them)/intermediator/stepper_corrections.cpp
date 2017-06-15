#include "stepper_corrections.h"

void enable_received() {
	digitalWriteFast(PASS_ENABLE, digitalReadFast(RECIEVE_ENABLE));

	#ifdef DEBUG_INTERRUPTS
		Serial.println("enable_received");
	#endif
}

void step_received() {
	digitalWriteFast(PASS_STEP, digitalReadFast(RECIEVE_STEP));

	#ifdef DEBUG_INTERRUPTS
		Serial.println("step_received");
	#endif
}

void dir_received() {
	digitalWriteFast(PASS_DIR, digitalReadFast(RECIEVE_DIR));

	#ifdef DEBUG_INTERRUPTS
		Serial.println("dir_received");
	#endif
}