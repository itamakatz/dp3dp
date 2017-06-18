#include "sixDOF_Tenssy3.h"

void sixDOF_Tenssy3::sixDOF_setup(float alpha) {

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::sixDOF_setup");
	#endif

	delay(sixDOF_CRITICAL_DELAY);

	memset(_average, 0, sizeof(_average));
	memset(_average_zero_offset, 0, sizeof(_average_zero_offset));

	memset(_angles_Euler, 0, sizeof(_angles_Euler));
	memset(_angles_Euler_zero_offset, 0, sizeof(_angles_Euler_zero_offset));

	memset(_c_array, 0, sizeof(_c_array));

	_alpha = alpha;

	for (int j = 0; j < 3; ++j) {
		_c_array[j] = Cyclic_array();
	}

	FsixDOF_Tenssy3 = FreeSixIMU_Tenssy3();
	FsixDOF_Tenssy3.init();
	_init_samples();
}

void sixDOF_Tenssy3::_init_samples(){

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::_init_samples");
	#endif

	for (double i = 0.0; i < CYC_ARRAY_SIZE_6DoF; ++i) {
		FsixDOF_Tenssy3.getEuler(_angles_Euler);
		for (int j = 0; j < 3; ++j){
			_c_array[j].insert(_angles_Euler[j]);
		}
	}

	for (int j = 0; j < 3; ++j) {
		_average[j] = _angles_Euler[j];
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

	for (int j = 0; j < 3; ++j){
		_c_array[j].insert(_angles_Euler[j]);
	}

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::sixDOF_loop - after insert");
	#endif

	_update_average();

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::sixDOF_loop - after _update_average");
	#endif
}

void sixDOF_Tenssy3::set_zero(){
	for (int j = 0; j < 3; ++j) {
		_average_zero_offset[j] = _average[j];
		_angles_Euler_zero_offset[j] = _angles_Euler[j];
	}
}

void sixDOF_Tenssy3::_update_average(){

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::_update_average");
	#endif

	// since the exponential window is recursive, set the first average value as an initial condition
	for (int j = 0; j < 3; ++j) {
		_average[j] = _c_array[j].get_cyc_array_single(0);
	}

	// note i = 1 due to the above
	for (int i = 1; i < CYC_ARRAY_SIZE_6DoF; ++i){
		for (int j = 0; j < 3; ++j) {
			_average[j] = _alpha * _average[j] + (1 - _alpha) * _c_array[j].get_cyc_array_single(i);
		}
	}
}

void sixDOF_Tenssy3::get_angles(float* angles_Euler) { 

	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::get_angles");
	#endif

	// FsixDOF_Tenssy3.getQ(q);
	// serialPrintFloatArr(q, 4);

	for (int j = 0; j < 3; ++j) {
		angles_Euler[j] = _angles_Euler[j] - _angles_Euler_zero_offset[j];
	}

	#ifdef DEBUG_PRINT_sixDOF_Tenssy3

		Serial.print("sixDOF_Tenssy3 - _angles_Euler: ");
		for (int j = 0; j < 3; ++j){
			Serial.print(_angles_Euler[j]);	
			Serial.print(", ");
		}

		Serial.println();

		Serial.print("sixDOF_Tenssy3 - angles_Euler: ");
		for (int j = 0; j < 3; ++j){
			Serial.print(angles_Euler[j]);	
			Serial.print(", ");
		}

		Serial.println();

	#endif
}

void sixDOF_Tenssy3::get_average(float* angles_average){
	
	#ifdef DEBUG_FUNC_FLOW__SIX_DFO__
		Serial.println("sixDOF_Tenssy3::get_average");
	#endif

	for (int j = 0; j < 3; ++j) {
		angles_average[j] = _average[j] - _average_zero_offset[j];
	}

	#ifdef DEBUG_PRINT_sixDOF_Tenssy3

		Serial.print("sixDOF_Tenssy3 - _average: ");
		for (int j = 0; j < 3; ++j){
			Serial.print(_average[j]);	
			Serial.print(", ");
		}

		Serial.println();

		Serial.print("sixDOF_Tenssy3 - angles_average: ");
		for (int j = 0; j < 3; ++j){
			Serial.print(angles_average[j]);	
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
	for (int j = 0; j < 3; ++j)
	{
		_average[j] = 0.0;
	}
	
	_init_samples();
}