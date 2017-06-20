#ifndef __Z_AXIS__
#define __Z_AXIS__

#include <Arduino.h>
#include "general_defs.h"

extern volatile bool enable_state;

void intermediator_setup_z();

void enable_received_z();
void dir_received_z();
void step_received_z();

#endif