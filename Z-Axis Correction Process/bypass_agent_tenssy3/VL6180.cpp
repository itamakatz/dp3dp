#include "VL6180.h"

VL6180::VL6180(float alpha){
	_average = 0;
	_alpha = alpha;
}


void VL6180::VL6180_setup()
{
	// _c_array = Cyclic_array(CYC_ARRAY_SIZE_VL6180);
	
	VL6180_sensor.init();

	#ifdef DEBUG_FUNC_FLOW_VL6180
		Serial.println("VL6180::VL6180_setup - after init()");
	#endif

	VL6180_sensor.configureDefault();

	#ifdef DEBUG_FUNC_FLOW_VL6180
		Serial.println("VL6180::VL6180_setup - after configureDefault()");
	#endif


	// Reduce range max convergence time and ALS integration
	// time to 30 ms and 50 ms, respectively, to allow 10 Hz
	// operation (as suggested by Table 6 ("Interleaved mode
	// limits (10 Hz operation)") in the datasheet).

	// VL6180_sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
	VL6180_sensor.writeReg(VL6180X_Tenssy3::SYSRANGE__MAX_CONVERGENCE_TIME, 30);

	#ifdef DEBUG_FUNC_FLOW_VL6180
		Serial.println("VL6180::VL6180_setup - after writeReg");
	#endif

	// VL6180_sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
	VL6180_sensor.writeReg16Bit(VL6180X_Tenssy3::SYSALS__INTEGRATION_PERIOD, 50);

	#ifdef DEBUG_FUNC_FLOW_VL6180
		Serial.println("VL6180::VL6180_setup - after writeReg16");
	#endif

	VL6180_sensor.setTimeout(500);

	#ifdef DEBUG_FUNC_FLOW_VL6180
		Serial.println("VL6180::VL6180_setup - after setTimeout");
	#endif

	 // stop continuous mode if already active
	VL6180_sensor.stopContinuous();

	#ifdef DEBUG_FUNC_FLOW_VL6180
		Serial.println("VL6180::VL6180_setup - after stopContinuous");
	#endif

	// in case stopContinuous() triggered a single-shot
	// measurement, wait for it to complete
	delay(VL6180_CRITICAL_DELAY);

	// start interleaved continuous mode with period of 100 ms
	VL6180_sensor.startInterleavedContinuous(100);

	#ifdef DEBUG_FUNC_FLOW_VL6180
		Serial.println("VL6180::VL6180_setup - after startInterleavedContinuous");
	#endif

	_init_samples();
}

uint16_t VL6180::_read_distance(){

	#ifdef DEBUG_FUNC_FLOW_VL6180
		Serial.println("VL6180::_read_distance");
	#endif

	#ifdef DEBUG_PRINT_VL6180
		Serial.println(VL6180_sensor.readRangeContinuousMillimeters());
		if (VL6180_sensor.timeoutOccurred()) { Serial.println("TIMEOUT"); }
	#endif

	return(VL6180_sensor.readRangeContinuousMillimeters());
}

void VL6180::_init_samples(){

	#ifdef DEBUG_FUNC_FLOW__VL6180__
		Serial.println("VL6180::_init_samples");
	#endif

	for (double i = 0.0; i < CYC_ARRAY_SIZE_VL6180; ++i) {
		_c_array.insert((float)_read_distance());
	}

	_average = _c_array.peek();
}

void VL6180::VL6180_loop() { 

	#ifdef DEBUG_FUNC_FLOW__VL6180__
		Serial.println("VL6180::VL6180_loop");
	#endif

	_c_array.insert((float)_read_distance());

	#ifdef DEBUG_FUNC_FLOW__VL6180__
		Serial.println("VL6180::VL6180_loop - after _read_distance + insert");
	#endif

	_update_average();

	#ifdef DEBUG_FUNC_FLOW__VL6180__
		Serial.println("VL6180::VL6180_loop - after _update_average");
	#endif
}

void VL6180::_update_average(){

	#ifdef DEBUG_FUNC_FLOW__VL6180__
		Serial.println("VL6180::_update_average");
	#endif

	// since the exponential window is recursive, set the first average value as an initial condition
	_average = _c_array.get_cyc_array_single(0);

	// note i = 1 due to the above
	for (int i = 1; i < CYC_ARRAY_SIZE_VL6180; ++i){
		_average = _alpha * _average + (1 - _alpha) * _c_array.get_cyc_array_single(i);
	}
}

float VL6180::get_average(){
	
	#ifdef DEBUG_FUNC_FLOW__VL6180__
		Serial.println("VL6180::get_average");
	#endif

	#ifdef DEBUG_PRINT_VL6180
		Serial.print("VL6180 - _average: ");
		Serial.print(_average[j]);	
		Serial.println(", ");
	#endif

	return _average;
}