#ifndef __CYCLIC_ARRAY__
#define __CYCLIC_ARRAY__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "general_defs.h"

#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
	#include <i2c_t3.h> // needed to use serial while debugging
#endif

class Cyclic_array {
private:
	// float _array[CYC_ARRAY_SIZE];
	float* _array;
	int _front = 0;
	int _itemCount = 0;

	Cyclic_array();
	
public:
	Cyclic_array(int array_size);
	~Cyclic_array();
	float peek();
	void get_cyc_array(float* get_array);
	float get_cyc_array_single(int i);
	bool isEmpty();
	bool isFull();
	int size();
	void insert(float data);
};

#endif

