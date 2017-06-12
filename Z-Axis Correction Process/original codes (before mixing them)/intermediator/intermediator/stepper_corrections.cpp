//
// Created by liav on 29/05/17.
//

#include "stepper_corrections.h"

int g_steps_gained_from_marlin = 0;
bool g_wrote_dir = false;


FORCE_INLINE void _enable() {
	digitalWriteFast(PASS_ENABLE, ENABLED);
}

FORCE_INLINE void _disable() {
	digitalWriteFast(PASS_ENABLE, DISABLED);
	g_wrote_dir = false;
}


void toggle_state(byte state){
	if (state == ENABLED){
		_enable();
	} else {
		_disable();
	}
}


void apply_steps(int num_of_steps) {
	#ifdef MULTIPLE_STEPS
		REPEAT(num_of_steps){
	#endif
			digitalWriteFast(PASS_STEP, LOW);
			digitalWriteFast(PASS_STEP, HIGH);
	#ifdef MULTIPLE_STEPS
		}
	#endif
	g_steps_gained_from_marlin += num_of_steps;
}


void set_direction() {
	if (g_wrote_dir) return;
	int dir = digitalReadFast(RECIEVE_DIR);
	digitalWriteFast(PASS_DIR, dir);
	g_wrote_dir = true;
}
