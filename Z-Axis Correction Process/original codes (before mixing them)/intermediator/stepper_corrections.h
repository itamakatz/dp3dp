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

#define RECIEVE_ENABLE    2 // Green
#define RECIEVE_STEP      3 // Yellow
#define RECIEVE_DIR       4 // Blue

#define PASS_ENABLE       6 // Green
#define PASS_STEP         7 // Yellow
#define PASS_DIR          8 // Blue

void enable_received();
void dir_received();
void step_received();

#endif //STEPPER_CORRECTIONS_H
