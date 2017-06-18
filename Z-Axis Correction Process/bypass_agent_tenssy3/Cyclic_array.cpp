#include "Cyclic_array.h"

float Cyclic_array::peek(){

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::peek");
	#endif

	return array[front];
}

void Cyclic_array::get_cyc_array(float* get_array){

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::get_cyc_array");
	#endif

	int index = front;

	for (int i = 0; i < C_ARRAY_SIZE; ++i)
	{
		get_array[i] = array[index];
		
		index--;

		if (index < 0) {
			index += C_ARRAY_SIZE;
		}
	}

	for (int j = 0; j < 3; ++j) {
	}
}

float Cyclic_array::get_cyc_array_single(int i){

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::get_cyc_array_single");
	#endif

	int index = front - i;

	if (index < 0) {
		index += C_ARRAY_SIZE;
	}
	return array[index];
}

bool Cyclic_array::isEmpty() {
	return itemCount == 0;
}

bool Cyclic_array::isFull() {
	return itemCount == C_ARRAY_SIZE - 1;
}

int Cyclic_array::size() {
	return itemCount;
}

void Cyclic_array::insert(float data) {

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::insert");
	#endif

	front++;

	// if not full increase count.
	// if front >= C_ARRAY_SIZE due to cyclicity, start from 0
	// regardless of the above, insert the data element
	if(!isFull()) {
		itemCount++;
	} else if (front >= C_ARRAY_SIZE) {
		front = 0;
	}

	array[front] = data;
}