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
	float array[C_ARRAY_SIZE][3];
	int front = 0;
	int rear = -1;
	int itemCount = 0;

public:
	Cyclic_array(){}
	void peek(float* get_array);
	void get_c_array(float* get_array, int i);
	bool isEmpty();
	bool isFull();
	int size();
	void insert(float* array_data);
};

#endif