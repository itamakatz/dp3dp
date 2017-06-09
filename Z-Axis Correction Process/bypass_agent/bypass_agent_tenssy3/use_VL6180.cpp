#include "use_VL6180.h"

void VL6180::VL6180_setup()
{
	sensor.init();
	sensor.configureDefault();

	// Reduce range max convergence time and ALS integration
	// time to 30 ms and 50 ms, respectively, to allow 10 Hz
	// operation (as suggested by Table 6 ("Interleaved mode
	// limits (10 Hz operation)") in the datasheet).
	sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
	sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

	sensor.setTimeout(500);

	 // stop continuous mode if already active
	sensor.stopContinuous();
	// in case stopContinuous() triggered a single-shot
	// measurement, wait for it to complete
	delay(VL6180_CRITICAL_DELAY);
	// start interleaved continuous mode with period of 100 ms
	sensor.startInterleavedContinuous(100);

}

int VL6180::read_distance()
{
	return(sensor.readRangeContinuousMillimeters());
}
