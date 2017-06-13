#include "stepper_corrections.h"

void setup(){

  	Serial.begin(9600);
  
  	pinMode(RECIEVE_ENABLE, INPUT);
  	pinMode(RECIEVE_STEP, INPUT);
  	pinMode(RECIEVE_DIR, INPUT);
  
  	pinMode(PASS_ENABLE, OUTPUT);
  	pinMode(PASS_STEP, OUTPUT);
  	pinMode(PASS_DIR, OUTPUT);
  
  	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), toggle, CHANGE);
  	attachInterrupt(digitalPinToInterrupt(RECIEVE_DIR), direction_received, CHANGE);
  	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received_rising, RISING);
  	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received_falling, FALLING);
  	// attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received, RISING);

	delay(3000);
}

void loop(){

	// digitalWriteFast(PASS_STEP, LOW);
  
	#ifdef DEBUG_LOOP
		Serial.print("state is: ");
	#endif
}
