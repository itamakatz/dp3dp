
#include "DRV8825.h"

#define ENABLE_PIN 2
#define STEP_PIN   3
#define DIR_PIN    4


DRV8825 stepper(2000, DIR_PIN, STEP_PIN, ENABLE_PIN);


void setup(){
	Serial.begin(9600);
	stepper.setMicrostep(1);
}
   

void loop(){
    stepper.rotate(233);
    delay(1000);
    stepper.rotate(-317);
    delay(1000);
    stepper.move(360);
    delay(1000);
    

}

