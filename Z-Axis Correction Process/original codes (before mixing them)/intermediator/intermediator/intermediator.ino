
#include "stepper_corrections.h"

#define DEBUG

volatile byte state = DISABLED;
Z_correction Z_C;



void toggle(){
	state = !state;
	Z_C.toggle(state);
}


void step_received(){
	if (state == ENABLED){
    Z_C.set_direction();
		Z_C.apply_steps(1);
	}
}



void setup(){

	Serial.begin(9600);

  Z_C.init();

	pinMode(RECIEVE_ENABLE, INPUT);
	pinMode(RECIEVE_STEP, INPUT);
	pinMode(RECIEVE_DIR, INPUT);

	pinMode(PASS_ENABLE, OUTPUT);
	pinMode(PASS_STEP, OUTPUT);
	pinMode(PASS_DIR, OUTPUT);

  digitalWrite(PASS_STEP, LOW);

	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), toggle, LOW);
	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received, RISING);

}


void loop(){

  if (state == DISABLED){
    // examine the sensor constantly. (according to _height_gained_from_marlin)
    // repair it.
  }

  #ifdef DEBUG
    Serial.print("state is: ");
    Serial.println(state ? "DISABLED": "ENABLED");  
  #endif

  digitalWriteFast(PASS_STEP, LOW);

//	Serial.println(state == DISABLED ? "DISABLED": "ENABLED");
//  digitalWrite(PASS_ENABLE, HIGH);
//  digitalWrite(PASS_DIR, HIGH);
//  digitalWrite(PASS_STEP, HIGH);
//  digitalWrite(PASS_ENABLE, LOW);
//  delay(1500);

}

