#include "Cyclic_array.h"

void Cyclic_array::peek(float* get_array){

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::peek");
	#endif

	for (int i = 0; i < 3; ++i)
	{
		get_array[i] = array[front][i];
	}
}

void Cyclic_array::get_c_array(float* get_array, int i){

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::get_c_array");
	#endif

	int index = front - i;

	if (index < 0) {
		index += C_ARRAY_SIZE;
	}
	for (int j = 0; j < 3; ++j) {
		get_array[j] = array[index][j];
	}
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

void Cyclic_array::insert(float* array_data) {

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::insert");
	#endif

	front++;

	if(!isFull()) {
		itemCount++;
	} else if (front >= C_ARRAY_SIZE) {
		front = 0;
	}

	for (int i = 0; i < 3; ++i)
	{
		array[front][i] = array_data[i];
	}
}