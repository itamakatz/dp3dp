//
// Created by liav on 29/05/17.
//

#ifndef STEPPER_CORRECTIONS_H
#define STEPPER_CORRECTIONS_H

#include <Arduino.h>

//#define DEBUG_INTERRUPTS
//#define DEBUG_LOOP

//#define MULTIPLE_STEPS

#define ENABLED LOW
#define DISABLED HIGH
#define DELAY_TIME 10
#define REPEAT(NUM_STEPS) for(int i = NUM_STEPS; i--;)
#define FORCE_INLINE __attribute__((always_inline)) inline

#define RECIEVE_ENABLE    2    // connected to Z_ENABLE_PIN 62 of the teensy - interrupt pin
#define RECIEVE_STEP      3    // connected to Z_STEP_PIN 46 of the teensy   - interrupt pin
#define RECIEVE_DIR       4    // connected to Z_DIR_PIN 48 of the teensy    - interrupt pin

#define PASS_ENABLE       8    // connected to ENABLE pin 2 of the stepper driver 
#define PASS_STEP        13    // connected to STEP pin 16 of the stepper driver 
#define PASS_DIR         12    // connected to DIR pin 19 of the stepper driver 


extern int g_steps_gained_from_marlin;

extern volatile byte state;

void toggle();			// interrupt isr
void step_received();	// interrupt isr
void direction_received();	// interrupt isr

void toggle_state(byte state);
void apply_steps(int num_of_steps);



#endif //STEPPER_CORRECTIONS_H
