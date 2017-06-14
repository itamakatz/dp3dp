#include "intermediator.h"

// const int RECIEVE_ENABLE = 2; // connected to Z_ENABLE_PIN 62 of the mega
// const int PASS_ENABLE = 8;		// connected to ENABLE pin 2 of the stepper driver 

volatile bool state = true;

void toggle_enabled_pin(){
	cli();
	state = !state;
	// digitalWrite(PASS_ENABLE, digitalRead(RECIEVE_ENABLE));
	digitalWrite(PASS_ENABLE, state);
	digitalWrite(7, !state);
	Serial.println("enable_FALLING");
	sei();
}

void intermediator_setup(){

	Serial.println("intermediator_setup");

	// pinMode(RECIEVE_ENABLE, INPUT_PULLUP);
	pinMode(RECIEVE_ENABLE, INPUT);
	pinMode(7, OUTPUT);

	pinMode(PASS_ENABLE, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), toggle_enabled_pin, CHANGE);
}

void intermediator_loop(){

	Serial.print("RECIEVE_ENABLE is:");
	Serial.println(digitalRead(RECIEVE_ENABLE));
	
	state = digitalRead(RECIEVE_ENABLE);
	digitalWrite(PASS_ENABLE, state);
	digitalWrite(7, !state);
	
	Serial.print("state is:");
	Serial.println(state);
}