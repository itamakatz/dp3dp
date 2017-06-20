#include "stepper.h"

DRV8825_teensy z_stepper(2000, PASS_DIR_Z, PASS_STEP_Z, PASS_ENABLE_Z);
Cyc_array_stepper c_array = Cyc_array_stepper();
int step_count = 0;
float average = 0;
float alpha = STEPPER_ALPHA;


void stepper_setup(){
	#ifdef DEBUG_FUNC_FLOW_STEPPER
		Serial.println(F("stepper - stepper_setup"));
	#endif
	
	z_stepper.setMicrostep(1);
	z_stepper.setRPM(120);

}

void stepper_add(float num){

	#ifdef DEBUG_FUNC_FLOW_STEPPER
		Serial.println(F("stepper - stepper_add"));
	#endif

	step_count += num * SETEPS_TO_MM;

	step_count /= 2;

	c_array.insert(step_count);

	if(c_array.isFull()){
		update_average();
	}

	#ifdef DEBUG_PRINTS_STEPPER
		Serial.print(F("stepper - step_count is :"));
		Serial.println(step_count);
	#endif
}

void update_average(){

	#ifdef DEBUG_FUNC_FLOW_SIX_DFO_
		Serial.println(F("stepper - update_average"));
	#endif

	// since the exponential window is recursive, set the first average value as an initial condition
	average = c_array.get_cyc_array_single(0);

	// note i = 1 due to the above
	for (int i = 1; i < CYC_ARRAY_SIZE_6DoF; ++i){
		average = alpha * average + (1 - alpha) * c_array.get_cyc_array_single(i);
	}
}

void stepper_loop(){

	#ifdef DEBUG_FUNC_FLOW_STEPPER
		Serial.println(F("stepper - stepper_loop"));
	#endif

	if(c_array.isFull()){
		digitalWriteFast(PASS_ENABLE_Z, LOW);		
		if (abs(step_count) >= STEPS_PER_LOOP){
			if (step_count >= 0) {
				z_stepper.move(STEPS_PER_LOOP);
				step_count -= STEPS_PER_LOOP;
			} else{
				z_stepper.move(-1*STEPS_PER_LOOP);
				step_count += STEPS_PER_LOOP;	
			}
		} else {
			z_stepper.move(step_count);
			step_count = 0;
		}
		digitalWriteFast(PASS_ENABLE_Z, HIGH);
	}
}