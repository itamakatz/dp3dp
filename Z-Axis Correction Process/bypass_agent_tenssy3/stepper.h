#ifndef __STEPPER__
#define __STEPPER__

#include <DRV8825_teensy.h>
#include "general_defs.h"

// 60[s/min] * 1000000[us/s] / microsteps / steps / rpm
// 60[s/min] * 1000000[us/s] / microsteps [micro-steps/step] / steps [step] / rpm [200 * step / min]
// => [us/min]
// => [(us * step)/(min * micro-steps)]
// => [us/(min * micro-steps)]
// => [us/(step* micro-steps)]

extern DRV8825_teensy z_stepper;

void stepper_setup();

void stepper_loop();

#endif