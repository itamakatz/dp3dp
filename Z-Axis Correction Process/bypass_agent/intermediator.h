#ifndef __intermediator__
#define __intermediator__

#include "pins.h"
#include "Arduino.h"


extern volatile bool state;

// void enable_FALLING();

// void enable_RISING();

void toggle_enabled_pin();

// void step_received();

void intermediator_setup();

void intermediator_loop();


#endif