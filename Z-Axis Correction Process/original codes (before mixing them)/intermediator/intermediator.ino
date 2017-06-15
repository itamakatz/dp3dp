#include "stepper_corrections.h"

void setup(){

	#ifdef SERIAL_INIT
		Serial.begin(250000);
	#endif

	pinMode(RECIEVE_ENABLE, INPUT);
	pinMode(RECIEVE_STEP, INPUT);
	pinMode(RECIEVE_DIR, INPUT);

	pinMode(PASS_ENABLE, OUTPUT);
	pinMode(PASS_STEP, OUTPUT);
	pinMode(PASS_DIR, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), toggle, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RECIEVE_DIR), direction_received, CHANGE);
	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received, CHANGE);

	delay(3000);
}

void loop(){
	#ifdef DEBUG_LOOP
		Serial.println("state is: ");
	#endif
}
