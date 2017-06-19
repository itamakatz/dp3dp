#ifndef __VL6180__
#define __VL6180__

#include <i2c_t3.h>
#include "VL6180X_Tenssy3.h"
#include "general_defs.h"
#include "Cyclic_array.h"

class VL6180 {
private:
	VL6180X_Tenssy3 _VL6180_sensor;
	Cyclic_array _c_array;

	float _average = 0;
	float _alpha = 0;

	void _init_samples();
	void _update_average();
	uint16_t _read_distance();

public:
	VL6180(){}
	void VL6180_setup(float alpha);
	float get_average();
	void VL6180_loop();
};

#endif