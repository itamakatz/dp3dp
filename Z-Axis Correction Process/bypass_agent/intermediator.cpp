#include "intermediator.h"

volatile bool state = false;


void toggle_enabled_pin(){
	cli();
	state = !state;
	digitalWrite(PASS_ENABLE, state);
	sei();
}


// void step_received(){
// 	cli();
// 	if (state){
// 		digitalWrite(PASS_DIR, digitalRead(RECIEVE_DIR));
// 		digitalWrite(PASS_STEP, digitalRead(RECIEVE_STEP));
// 	}
// 	sei();
// }

void intermediator_setup(){

	Serial.println("intermediator_setup");


	pinMode(RECIEVE_ENABLE, INPUT);
	// pinMode(RECIEVE_STEP, INPUT);
	// pinMode(RECIEVE_DIR, INPUT);

	pinMode(PASS_ENABLE, OUTPUT);
	// pinMode(PASS_STEP, OUTPUT);
	// pinMode(PASS_DIR, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), toggle_enabled_pin, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received, CHANGE);
}

void intermediator_loop(){

	// if (state){
	// 	digitalWrite(PASS_DIR, digitalRead(RECIEVE_DIR));
	// 	digitalWrite(PASS_STEP, digitalRead(RECIEVE_STEP));
	// }
	// digitalWrite(PASS_DIR, digitalRead(RECIEVE_DIR));
	// digitalWrite(PASS_STEP, digitalRead(RECIEVE_STEP));

}