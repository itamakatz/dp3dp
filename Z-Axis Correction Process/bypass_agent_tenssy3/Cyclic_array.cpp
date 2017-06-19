#include "Cyclic_array.h"

Cyclic_array::Cyclic_array(int array_size){
	_array = (float*) calloc(array_size ,sizeof(float));
}

Cyclic_array::~Cyclic_array(){
	free(_array);
}


float Cyclic_array::peek(){

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::peek");
	#endif

	return _array[_front];
}

void Cyclic_array::get_cyc_array(float* get_array){

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::get_cyc_array");
	#endif

	int index = _front;

	for (int i = 0; i < CYC_ARRAY_SIZE; ++i)
	{
		get_array[i] = _array[index];
		
		index--;

		if (index < 0) {
			index += CYC_ARRAY_SIZE;
		}
	}

	for (int j = 0; j < 3; ++j) {
	}
}

float Cyclic_array::get_cyc_array_single(int i){

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::get_cyc_array_single");
	#endif

	int index = _front - i;

	if (index < 0) {
		index += CYC_ARRAY_SIZE;
	}
	return _array[index];
}

bool Cyclic_array::isEmpty() {
	return _itemCount == 0;
}

bool Cyclic_array::isFull() {
	return _itemCount == CYC_ARRAY_SIZE - 1;
}

int Cyclic_array::size() {
	return _itemCount;
}

void Cyclic_array::insert(float data) {

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println("Cyclic_array::insert");
	#endif

	#ifdef DEBUG_CYCLIC_ARRAY
		Serial.print("_front before increment");
		Serial.println(_front);
	#endif

	_front++;

	#ifdef DEBUG_CYCLIC_ARRAY
		Serial.print("_front after increment");
		Serial.println(_front);
	#endif

	// if not full increase count.
	// if _front >= CYC_ARRAY_SIZE due to cyclicity, start from 0
	// regardless of the above, insert the data element
	if(!isFull()) {

		#ifdef DEBUG_CYCLIC_ARRAY
			Serial.println("inside if(!isFull())");
		#endif

		_itemCount++;

		#ifdef DEBUG_CYCLIC_ARRAY
			Serial.print("_itemCount is");
			Serial.println(_itemCount);
		#endif

	} else if (_front >= CYC_ARRAY_SIZE) {
		
		#ifdef DEBUG_CYCLIC_ARRAY
			Serial.println("inside else if (_front >= CYC_ARRAY_SIZE)");
		#endif

		_front = 0;
	}

	#ifdef DEBUG_CYCLIC_ARRAY
		Serial.println("before _array[_front] = data");
	#endif

	#ifdef DEBUG_CYCLIC_ARRAY
		Serial.print("sizeof(_array) is :");
		Serial.println(sizeof(_array));
	#endif

	_array[_front] = data;
}