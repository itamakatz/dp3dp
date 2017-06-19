#include "intermediator.h"
#include "general_defs.h"

void intermediator_setup(){

	cli();

	pinMode(RECIEVE_ENABLE, INPUT);
	pinMode(RECIEVE_STEP, INPUT);
	pinMode(RECIEVE_DIR, INPUT);

	pinMode(PASS_ENABLE, OUTPUT);
	pinMode(PASS_STEP, OUTPUT);
	pinMode(PASS_DIR, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), enable_received, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RECIEVE_DIR), dir_received, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received, CHANGE);

	digitalWriteFast(PASS_ENABLE, digitalReadFast(RECIEVE_ENABLE));
	digitalWriteFast(PASS_DIR, digitalReadFast(RECIEVE_DIR));

	sei();

	// delay(INTERMEDIATOR_DELAY);
}

void enable_received() {
	digitalWriteFast(PASS_ENABLE, digitalReadFast(RECIEVE_ENABLE));

	#ifdef DEBUG_INTERRUPTS
		Serial.println(F("enable_received"));
	#endif
}

void step_received() {
	digitalWriteFast(PASS_STEP, digitalReadFast(RECIEVE_STEP));

	#ifdef DEBUG_INTERRUPTS
		Serial.println(F("step_received"));
	#endif
}

void dir_received() {
	digitalWriteFast(PASS_DIR, digitalReadFast(RECIEVE_DIR));

	#ifdef DEBUG_INTERRUPTS
		Serial.println(F("dir_received"));
	#endif
}