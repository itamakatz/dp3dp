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
  	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received, RISING);
  	attachInterrupt(digitalPinToInterrupt(RECIEVE_DIR), direction_received, CHANGE);

	delay(3000);

}

void loop(){

	
	// examine the sensor constantly. (according to _steps_gained_from_marlin)
	// repair it.
	
	keep_step_low();
  
	#ifdef DEBUG_LOOP
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");  
	#endif
}

inline void keep_step_low(){
	digitalWriteFast(PASS_STEP, LOW);
}

