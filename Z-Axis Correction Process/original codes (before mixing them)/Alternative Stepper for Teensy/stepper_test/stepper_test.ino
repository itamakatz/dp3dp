
#include "DRV8825.h"

#define ENABLE_PIN 2
#define STEP_PIN   3
#define DIR_PIN    4
#define E0		   5
#define E1		   6
#define E2		   7

// 60[s/min] * 1000000[us/s] / microsteps / steps / rpm
// 60[s/min] * 1000000[us/s] / microsteps [micro-steps/step] / steps [step] / rpm [200 * step / min]
// => [us/min]
// => [(us * step)/(min * micro-steps)]
// => [us/(min * micro-steps)]
// => [us/(step* micro-steps)]
// 

// DRV8825 stepper(2000, DIR_PIN, STEP_PIN, ENABLE_PIN);
DRV8825 stepper1(125, DIR_PIN, STEP_PIN, ENABLE_PIN, E0, E1, E2);
DRV8825 stepper2(80, DIR_PIN, STEP_PIN, ENABLE_PIN, E0, E1, E2);


void setup(){
	Serial.begin(9600);
	stepper1.setMicrostep(16);
}
 
void loop(){

    for (int i = 1; i < 200; ++i)
    {
        stepper1.setRPM(i);
        Serial.print("setRPM to ");
        Serial.println(i);
        Serial.print("step_pulse = ");
        Serial.println(stepper1.get_step_pulse());
        stepper1.rotate(40);
        delay(300);
    }
/*
    stepper1.rotate(233);
    delay(1000);
    stepper1.rotate(-317);
    delay(1000);
    stepper1.move(360);*/
}  

// void setup2(){
// 	Serial.begin(9600);
// }

// void loop2(){

// 	stepper1.setMicrostep(16);
    
//     stepper1.rotate(233);
//     delay(1000);
//     stepper1.rotate(-317);
//     delay(1000);
//     stepper1.move(360);
//     delay(1000);

// 	stepper2.setMicrostep(32);

//     stepper2.rotate(233);
//     delay(1000);
//     stepper2.rotate(-317);
//     delay(1000);
//     stepper2.move(360);
//     delay(1000);
// }