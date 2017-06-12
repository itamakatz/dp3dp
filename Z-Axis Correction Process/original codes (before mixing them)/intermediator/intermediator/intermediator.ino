
#include "stepper_corrections.h"

//#define DEBUG_INTERRUPTS
//#define DEBUG_LOOP

volatile byte state = ENABLED;


void toggle(){
	#ifdef DEBUG_INTERRUPTS
		Serial.println("in toggle");
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");  
	#else
		if (digitalReadFast(RECIEVE_ENABLE) == HIGH){
			state = DISABLED;
			toggle_state(DISABLED);
		} else {
			state = ENABLED;
			toggle_state(ENABLED);
		}	  	
	#endif
}


void step_received(){ // it's marlin's turn
	#ifdef DEBUG_INTERRUPTS
		Serial.println("in step_recieved");
		Serial.print("state is: ");
		Serial.println(state ? "DISABLED": "ENABLED");
	#else
		if (state == ENABLED){
			set_direction();
		  	apply_steps(1);
		}
	#endif
}



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

