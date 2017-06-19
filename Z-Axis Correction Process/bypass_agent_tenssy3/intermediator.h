#include <Arduino.h>
#include "general_defs.h"

extern volatile bool enable_state;


void intermediator_setup();

void enable_received();
void dir_received();
void step_received();