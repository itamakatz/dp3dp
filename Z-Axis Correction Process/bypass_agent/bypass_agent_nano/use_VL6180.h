#ifndef __USE_VL6180__
#define __USE_VL6180__

#include <Wire.h>
#include "VL6180X.h"

class VL6180 {
private:
	// float angles_Euler[3]
	VL6180X sensor; // FreeSixIMU object

public:
	VL6180(){}
	void VL6180_setup();
	int read_distance();
};

#endif