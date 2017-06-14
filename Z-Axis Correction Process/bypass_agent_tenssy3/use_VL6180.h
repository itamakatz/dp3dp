#ifndef __USE_VL6180__
#define __USE_VL6180__

#include <i2c_t3.h>
#include "VL6180X_Tenssy3.h"
#include "general_defs.h"

class VL6180 {
private:
	VL6180X_Tenssy3 sensor;
public:
	VL6180(){}
	void VL6180_setup();
	uint16_t read_distance();
};

#endif