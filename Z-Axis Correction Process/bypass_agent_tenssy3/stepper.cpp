#include "stepper.h"

DRV8825_teensy z_stepper(125, PASS_DIR, PASS_STEP, PASS_ENABLE);

void stepper_setup(){
	#ifdef DEBUG_FUNC_FLOW_STEPPER
		Serial.println(F("stepper - stepper_setup"));
	#endif
	
	z_stepper.setMicrostep(16);
}

void stepper_loop(){

	#ifdef DEBUG_FUNC_FLOW_STEPPER
		Serial.println(F("stepper - stepper_loop"));
	#endif
		
	z_stepper.rotate(180);
	// delay(1000);
	// z_stepper.rotate(-317);
	// delay(1000);
	// z_stepper.move(360);


	// for (int i = 1; i < 200; ++i)
	// {
	// 	z_stepper.setRPM(i);
	// 	Serial.print("setRPM to ");
	// 	Serial.println(i);
	// 	Serial.print("step_pulse = ");
	// 	Serial.println(z_stepper.get_step_pulse());
	// 	z_stepper.rotate(40);
	// 	delay(300);
	// }
}