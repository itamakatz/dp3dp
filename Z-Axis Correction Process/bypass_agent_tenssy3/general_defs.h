#ifndef __GENETAL_DEFS__
#define __GENETAL_DEFS__

#include "debug_def.h"
#include "pins.h"

// =========================== z-correction parameters =========================== //

#define CYC_ARRAY_SIZE_6DoF 10
#define CYC_ARRAY_SIZE_VL6180 200
#define CYC_ARRAY_SIZE_STEPPER 50

#define sixDOF_ALPHA (float)0.1
#define VL6180_ALPHA (float)0.001
#define STEPPER_ALPHA (float)0.01

#define SETEPS_PER_REVOLUTION 3300
#define STEPS_PER_LOOP 500
#define SETEPS_TO_MM 50

#define METRO_STEPPER_INTERVAL 100

// ================================================================================= //

// Initial delays needed for the sensors to stabilize
#define sixDOF_CRITICAL_DELAY 60
#define VL6180_CRITICAL_DELAY 300
#define INTERMEDIATOR_DELAY 3000

// Other delays
#define MAIN_LOOP_CRITICAL_DELAY 60

// Blincking led timing
#define METRO_HIGH_INTERVAL 250
#define METRO_LOW_INTERVAL 1000
#define METRO_PRINT_INTERVAL 1000
#define METRO_BIG_NUM 50000

#define ENABLED LOW
#define DISABLED HIGH

#define RIGHT 1
#define LEFT -1

#define R_AXIS_STEPS_PER_MM 160

// #define DISTANCE_CONVERSION_TIME 4000
#define DISTANCE_CONVERSION_TIME 500

#endif