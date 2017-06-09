#ifndef __GENETAL_DEFS__
#define __GENETAL_DEFS__

#include "debug_def.h"
#include "pins.h"

#define C_ARRAY_SIZE 30
#define ENABLE_NORMAL_PRINTS

#ifndef ENABLE_NORMAL_PRINTS
	#ifndef DISABLE_NORMAL_PRINTS
		#undef DISABLE_NORMAL_PRINTS
	#endif
#endif

// Initial delays needed for the sensors to stabilize
#define sixDOF_CRITICAL_DELAY 20
#define VL6180_CRITICAL_DELAY 20

// Other delays
#define MAIN_LOOP_CRITICAL_DELAY 60
#define LONG_DELAY 1500



#endif