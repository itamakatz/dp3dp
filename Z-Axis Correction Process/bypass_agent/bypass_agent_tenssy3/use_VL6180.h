#ifndef __USE_VL6180__
#define __USE_VL6180__

#include <i2c_t3.h>
#include "VL6180X.h"
#include "general_defs.h"

class VL6180 {
private:
	VL6180X sensor; // FreeSixIMU object

public:
	VL6180(){}
	void VL6180_setup();
	int read_distance();
};

#endif