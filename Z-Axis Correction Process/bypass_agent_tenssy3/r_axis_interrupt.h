#ifndef __R_AXIS__
#define __R_AXIS__

#include <Arduino.h>
#include "general_defs.h"

extern volatile int r_num_of_steps;

void intermediator_setup_r();

void dir_received_r();
void step_received_r();

#endif