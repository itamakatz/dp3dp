#include "Cyc_array_stepper.h"

Cyc_array_stepper::Cyc_array_stepper(){
	memset(_array, 0, sizeof(_array));
	_front = 0;
	_itemCount = 0;
}

float Cyc_array_stepper::peek(){

	#ifdef DEBUG_FUNC_FLOW_Cyc_array_stepper_
		Serial.println(F("Cyc_array_stepper::peek"));
	#endif

	return _array[_front];
}

void Cyc_array_stepper::get_cyc_array(float* get_array){

	#ifdef DEBUG_FUNC_FLOW_Cyc_array_stepper_
		Serial.println(F("Cyc_array_stepper::get_cyc_array"));
	#endif

	int index = _front;

	for (int i = 0; i < CYC_ARRAY_SIZE_STEPPER; ++i)
	{
		get_array[i] = _array[index];
		
		index--;

		if (index < 0) {
			index += CYC_ARRAY_SIZE_STEPPER;
		}
	}
}

float Cyc_array_stepper::get_cyc_array_single(int i){

	#ifdef DEBUG_FUNC_FLOW_Cyc_array_stepper_
		Serial.println(F("Cyc_array_stepper::get_cyc_array_single"));
	#endif

	int index = _front - i;

	if (index < 0) {
		index += CYC_ARRAY_SIZE_STEPPER;
	}
	return _array[index];
}

bool Cyc_array_stepper::isEmpty() {
	return _itemCount == 0;
}

bool Cyc_array_stepper::isFull() {
	return _itemCount == CYC_ARRAY_SIZE_STEPPER - 1;
}

void Cyc_array_stepper::insert(float data) {

	#ifdef DEBUG_FUNC_FLOW_Cyc_array_stepper_
		Serial.println(F("Cyc_array_stepper::insert"));
	#endif

	#ifdef DEBUG_PRINTS_Cyc_array_stepper
		Serial.print(F("_front before increment"));
		Serial.println(_front);
	#endif

	_front++;

	#ifdef DEBUG_PRINTS_Cyc_array_stepper
		Serial.print(F("_front after increment"));
		Serial.println(_front);
	#endif

	// if not full increase count.
	// if _front >= CYC_ARRAY_SIZE_STEPPER due to cyclicity, start from 0
	// regardless of the above, insert the data element
	if(!isFull()) {

		#ifdef DEBUG_PRINTS_Cyc_array_stepper
			Serial.println(F("inside if(!isFull())"));
		#endif

		_itemCount++;

		#ifdef DEBUG_PRINTS_Cyc_array_stepper
			Serial.print(F("_itemCount is"));
			Serial.println(_itemCount);
		#endif

	} else if (_front >= CYC_ARRAY_SIZE_STEPPER) {
		
		#ifdef DEBUG_PRINTS_Cyc_array_stepper
			Serial.println(F("inside else if (_front >= CYC_ARRAY_SIZE_STEPPER)"));
		#endif

		_front = 0;
	}

	#ifdef DEBUG_PRINTS_Cyc_array_stepper
		Serial.println(F("before _array[_front] = data"));
	#endif

	#ifdef DEBUG_PRINTS_Cyc_array_stepper
		Serial.print(F("sizeof(_array) is :"));
		Serial.println(sizeof(_array));
	#endif

	_array[_front] = data;
}