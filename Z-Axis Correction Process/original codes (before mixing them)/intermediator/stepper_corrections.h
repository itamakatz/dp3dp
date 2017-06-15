#ifndef STEPPER_CORRECTIONS_H
#define STEPPER_CORRECTIONS_H

#include <Arduino.h>

// #define DEBUG_INTERRUPTS
// #define DEBUG_LOOP
// #define SERIAL_INIT

#ifdef DEBUG_INTERRUPTS
	#ifndef SERIAL_INIT
		#define SERIAL_INIT
	#endif
#endif

#ifdef DEBUG_LOOP
	#ifndef SERIAL_INIT
		#define SERIAL_INIT
	#endif
#endif

#define ENABLED LOW
#define DISABLED HIGH

#define RECIEVE_ENABLE    2    // connected to Z_ENABLE_PIN 62 of the teensy - interrupt pin
#define RECIEVE_STEP      3    // connected to Z_STEP_PIN 46 of the teensy   - interrupt pin
#define RECIEVE_DIR       4    // connected to Z_DIR_PIN 48 of the teensy    - interrupt pin

#define PASS_ENABLE       6    // connected to ENABLE pin 2 of the stepper driver 
#define PASS_STEP         7    // connected to STEP pin 16 of the stepper driver 
#define PASS_DIR          8    // connected to DIR pin 19 of the stepper driver 

void toggle();			// interrupt isr
void direction_received();	// interrupt isr
void step_received();	// interrupt isr

#endif //STEPPER_CORRECTIONS_H
