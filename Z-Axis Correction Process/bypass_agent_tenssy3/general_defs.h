#ifndef __GENETAL_DEFS__
#define __GENETAL_DEFS__

#include "debug_def.h"
#include "pins.h"

#define C_ARRAY_SIZE 30

#define ENABLE_NORMAL_PRINTS
#ifdef ENABLE_NORMAL_PRINTS
	#ifdef DISABLE_NORMAL_PRINTS
		#undef DISABLE_NORMAL_PRINTS
	#endif
#endif

// Initial delays needed for the sensors to stabilize
#define sixDOF_CRITICAL_DELAY 60
#define VL6180_CRITICAL_DELAY 300
#define INTERMEDIATOR_DELAY 3000

// Other delays
#define MAIN_LOOP_CRITICAL_DELAY 60

// Blincking led timing
#define METRO_HIGH_INTERVAL 250
#define METRO_LOW_INTERVAL 1000

#endif