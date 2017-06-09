
#include "stepper_corrections.h"

//#define DEBUG_INTERRUPTS
//#define DEBUG_LOOP

volatile byte state = DISABLED;



void toggle(){
    #ifdef DEBUG_INTERRUPTS
        Serial.println("in toggle");
        Serial.print("state is: ");
        Serial.println(state ? "DISABLED": "ENABLED");  
    #else
      	state = !state;
      	toggle(state);
    #endif
}


void step_received(){
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
  
  	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), toggle, LOW);
  	attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received, RISING);

    delay(3000);

}


void loop(){

    if (state == DISABLED){
        // examine the sensor constantly. (according to _steps_gained_from_marlin)
        // repair it.
    }
  
    #ifdef DEBUG_LOOP
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

