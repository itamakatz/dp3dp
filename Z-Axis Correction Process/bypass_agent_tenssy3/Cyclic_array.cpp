#include "Cyclic_array.h"

float Cyclic_array::peek(){

	#ifdef DEBUG_FUNC_FLOW_CYCLIC_ARRAY_
		Serial.println(F("Cyclic_array::peek"));
	#endif

	return _array[_front];
}

void Cyclic_array::get_cyc_array(float* get_array){

	#ifdef DEBUG_FUNC_FLOW_CYCLIC_ARRAY_
		Serial.println(F("Cyclic_array::get_cyc_array"));
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
}

float Cyclic_array::get_cyc_array_single(int i){

	#ifdef DEBUG_FUNC_FLOW_CYCLIC_ARRAY_
		Serial.println(F("Cyclic_array::get_cyc_array_single"));
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

void Cyclic_array::insert(float data) {

	#ifdef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		Serial.println(F("Cyclic_array::insert"));
	#endif

	#ifdef DEBUG_CYCLIC_ARRAY
		Serial.print(F("_front before increment"));
		Serial.println(_front);
	#endif

	_front++;

	#ifdef DEBUG_CYCLIC_ARRAY
		Serial.print(F("_front after increment"));
		Serial.println(_front);
	#endif

	// if not full increase count.
	// if _front >= CYC_ARRAY_SIZE due to cyclicity, start from 0
	// regardless of the above, insert the data element
	if(!isFull()) {

		#ifdef DEBUG_CYCLIC_ARRAY
			Serial.println(F("inside if(!isFull())"));
		#endif

		_itemCount++;

		#ifdef DEBUG_CYCLIC_ARRAY
			Serial.print(F("_itemCount is"));
			Serial.println(_itemCount);
		#endif

	} else if (_front >= CYC_ARRAY_SIZE) {
		
		#ifdef DEBUG_CYCLIC_ARRAY
			Serial.println(F("inside else if (_front >= CYC_ARRAY_SIZE)"));
		#endif

		_front = 0;
	}

	#ifdef DEBUG_CYCLIC_ARRAY
		Serial.println(F("before _array[_front] = data"));
	#endif

	#ifdef DEBUG_CYCLIC_ARRAY
		Serial.print(F("sizeof(_array) is :"));
		Serial.println(sizeof(_array));
	#endif

	_array[_front] = data;
}