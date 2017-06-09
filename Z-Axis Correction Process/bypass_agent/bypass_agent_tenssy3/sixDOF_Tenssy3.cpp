#include "sixDOF_Tenssy3.h"

#define sixDOF_CRITICAL_DELAY 20

void sixDOF_Tenssy3::sixDOF_setup(float alpha) {

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::sixDOF_setup");
	#endif

	delay(sixDOF_CRITICAL_DELAY);

	memset(_average,0,sizeof(_average));
	memset(_angles_Euler,0,sizeof(_angles_Euler));
	memset(_c_array_elements,0,sizeof(_c_array_elements));

	_alpha = alpha;

	c_array = Cyclic_array();
	FsixDOF_Tenssy3 = FreeSixIMU_Tenssy3();
	FsixDOF_Tenssy3.init();
	_init_samples();
}

void sixDOF_Tenssy3::_init_samples(){

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::_init_samples");
	#endif

	for (double j = 0.0; j < C_ARRAY_SIZE; ++j)
	{
		FsixDOF_Tenssy3.getEuler(_angles_Euler);
		c_array.insert(_angles_Euler);
	}

	for (int i = 0; i < 3; ++i)
	{
		_average[i] = _angles_Euler[i];
	}
}

void sixDOF_Tenssy3::sixDOF_loop() { 

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::sixDOF_loop");
	#endif

	FsixDOF_Tenssy3.getEuler(_angles_Euler);

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::sixDOF_loop - after getEuler");
	#endif

	c_array.insert(_angles_Euler);

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::sixDOF_loop - after insert");
	#endif

	update_average();

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::sixDOF_loop - after update_average");
	#endif
}

void sixDOF_Tenssy3::update_average(){

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::update_average");
	#endif

	for (int i = 0; i < C_ARRAY_SIZE; ++i){

		c_array.get_c_array(&_c_array_elements[0], i);

		for (int j = 0; j < 3; ++j) {
			_average[j] = _alpha * _average[j] + (1 - _alpha) * _c_array_elements[j] ;
		}
	}
}

void sixDOF_Tenssy3::get_angles(float* angles_Euler) { 

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::get_angles");
	#endif

	// FsixDOF_Tenssy3.getQ(q);
	// serialPrintFloatArr(q, 4);

	angles_Euler[0] = _angles_Euler[0];
	angles_Euler[1] = _angles_Euler[1];
	angles_Euler[2] = _angles_Euler[2];

	#ifdef DEBUG_PRINT_sixDOF_Tenssy3

		Serial.print("sixDOF_Tenssy3 - _angles_Euler: ");
		for (int i = 0; i < 3; ++i){
			Serial.print(_angles_Euler[i]);	
			Serial.print(", ");
		}

		Serial.println();

		Serial.print("sixDOF_Tenssy3 - angles_Euler: ");
		for (int i = 0; i < 3; ++i){
			Serial.print(angles_Euler[i]);	
			Serial.print(", ");
		}

		Serial.println();

	#endif
}

void sixDOF_Tenssy3::get_average(float* angles_average){
	
	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::get_average");
	#endif

	angles_average[0] = _average[0];
	angles_average[1] = _average[1];
	angles_average[2] = _average[2];

	#ifdef DEBUG_PRINT_sixDOF_Tenssy3

		Serial.print("sixDOF_Tenssy3 - _average: ");
		for (int i = 0; i < 3; ++i){
			Serial.print(_average[i]);	
			Serial.print(", ");
		}

		Serial.println();

		Serial.print("sixDOF_Tenssy3 - angles_average: ");
		for (int i = 0; i < 3; ++i){
			Serial.print(angles_average[i]);	
			Serial.print(", ");
		}

		Serial.println();

	#endif	
}

void sixDOF_Tenssy3::calibrate(){

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::calibrate");
	#endif

	FsixDOF_Tenssy3.gyro.zeroCalibrate(128,5);
	for (int i = 0; i < 3; ++i)
	{
		_average[i] = 0.0;
	}
	
	_init_samples();
}