#include "z_axis_interrupt.h"
#include "general_defs.h"

volatile bool enable_state;

void intermediator_setup_z(){

	cli();

	pinMode(RECIEVE_ENABLE_Z, INPUT);
	pinMode(RECIEVE_STEP_Z, INPUT);
	pinMode(RECIEVE_DIR_Z, INPUT);

	pinMode(PASS_ENABLE_Z, OUTPUT);
	pinMode(PASS_STEP_Z, OUTPUT);
	pinMode(PASS_DIR_Z, OUTPUT);

	// attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE_Z), enable_received_z, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RECIEVE_DIR_Z), dir_received_z, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP_Z), step_received_z, CHANGE);

	digitalWriteFast(PASS_ENABLE_Z, digitalReadFast(RECIEVE_ENABLE_Z));
	digitalWriteFast(PASS_DIR_Z, digitalReadFast(RECIEVE_DIR_Z));

	// enable_state = digitalReadFast(RECIEVE_ENABLE_Z);
	enable_state = DISABLED;
	digitalWriteFast(PASS_ENABLE_Z, enable_state);

	sei();

	// delay(INTERMEDIATOR_DELAY);
}

// void enable_received_z() {
// 	enable_state = digitalReadFast(RECIEVE_ENABLE_Z);
// 	digitalWriteFast(PASS_ENABLE_Z, enable_state);

// 	#ifdef DEBUG_INTERRUPTS_Z
// 		Serial.println(F("enable_received_z"));
// 	#endif
// }

void step_received_z() {
	digitalWriteFast(PASS_ENABLE_Z, ENABLED);
	// enable_state = ENABLED;
	digitalWriteFast(PASS_STEP_Z, digitalReadFast(RECIEVE_STEP_Z));
	digitalWriteFast(PASS_ENABLE_Z, DISABLED);
	// enable_state = DISABLED;

	#ifdef DEBUG_INTERRUPTS_Z
		Serial.println(F("step_received_z"));
	#endif
}

void dir_received_z() {
	digitalWriteFast(PASS_ENABLE_Z, ENABLED);
	// enable_state = ENABLED;
	digitalWriteFast(PASS_DIR_Z, digitalReadFast(RECIEVE_DIR_Z));
	// enable_state = DISABLED;
	digitalWriteFast(PASS_ENABLE_Z, DISABLED);

	#ifdef DEBUG_INTERRUPTS_Z
		Serial.println(F("dir_received_z"));
	#endif
}