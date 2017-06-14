#include "use_VL6180.h"

void VL6180::VL6180_setup()
{
	sensor.init();

	#ifdef DEBUG_FUNC_FLOW__USE_VL6180
		Serial.println("VL6180::VL6180_setup - after init()");
	#endif

	sensor.configureDefault();


	#ifdef DEBUG_FUNC_FLOW__USE_VL6180
		Serial.println("VL6180::VL6180_setup - after configureDefault()");
	#endif


	// Reduce range max convergence time and ALS integration
	// time to 30 ms and 50 ms, respectively, to allow 10 Hz
	// operation (as suggested by Table 6 ("Interleaved mode
	// limits (10 Hz operation)") in the datasheet).

	// sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
	sensor.writeReg(VL6180X_Tenssy3::SYSRANGE__MAX_CONVERGENCE_TIME, 30);

	#ifdef DEBUG_FUNC_FLOW__USE_VL6180
		Serial.println("VL6180::VL6180_setup - after writeReg");
	#endif

	// sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
	sensor.writeReg16Bit(VL6180X_Tenssy3::SYSALS__INTEGRATION_PERIOD, 50);

	#ifdef DEBUG_FUNC_FLOW__USE_VL6180
		Serial.println("VL6180::VL6180_setup - after writeReg16");
	#endif

	sensor.setTimeout(500);

	#ifdef DEBUG_FUNC_FLOW__USE_VL6180
		Serial.println("VL6180::VL6180_setup - after setTimeout");
	#endif

	 // stop continuous mode if already active
	sensor.stopContinuous();

	#ifdef DEBUG_FUNC_FLOW__USE_VL6180
		Serial.println("VL6180::VL6180_setup - after stopContinuous");
	#endif

	// in case stopContinuous() triggered a single-shot
	// measurement, wait for it to complete
	delay(VL6180_CRITICAL_DELAY);
	// start interleaved continuous mode with period of 100 ms
	sensor.startInterleavedContinuous(100);

	#ifdef DEBUG_FUNC_FLOW__USE_VL6180
		Serial.println("VL6180::VL6180_setup - after startInterleavedContinuous");
	#endif
}

uint16_t VL6180::read_distance()
{

	#ifdef DEBUG_FUNC_FLOW__USE_VL6180
		Serial.println("VL6180::read_distance");
	#endif

	#ifdef DEBUG_PRINT_USE_VL6180
		Serial.println(sensor.readRangeContinuousMillimeters());
		if (sensor.timeoutOccurred()) { Serial.println("TIMEOUT"); }
	#endif

	return(sensor.readRangeContinuousMillimeters());
}
