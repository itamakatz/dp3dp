#ifndef __Cyc_array_VL6180__
#define __Cyc_array_VL6180__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "general_defs.h"

#ifdef DEBUG_FUNC_FLOW_CYCLIC_ARRAY_
	#include <i2c_t3.h> // needed to use serial while debugging
#endif

class Cyc_array_VL6180 {
private:
	float _array[CYC_ARRAY_SIZE_VL6180];
	int _front;
	int _itemCount;

public:
	Cyc_array_VL6180();
	float peek();
	void get_cyc_array(float* get_array);
	float get_cyc_array_single(int i);
	bool isEmpty();
	bool isFull();
	void insert(float data);
};

#endif

