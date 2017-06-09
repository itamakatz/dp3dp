//
// Created by liav on 29/05/17.
//

#include "stepper_corrections.h"


Z_correction::Z_correction(){}

Z_correction::~Z_correction(){}

void Z_correction::toggle(byte state){
  state == ENABLED  ? _enable(): _disable();
}

void Z_correction::init(){
  _steps_gained_from_marlin = 0;
  _wrote_dir = false;
}

void Z_correction::_enable() {
    digitalWriteFast(PASS_ENABLE, ENABLED);
}

void Z_correction::_disable() {
    digitalWriteFast(PASS_ENABLE, DISABLED);
    _wrote_dir = false;
}


void Z_correction::apply_steps(int num_of_steps) {
    REPEAT(num_of_steps){
        digitalWriteFast(PASS_STEP, LOW);
        digitalWriteFast(PASS_STEP, HIGH);
        //delayMicroseconds(DELAY_TIME);
    }
    _steps_gained_from_marlin += num_of_steps;
}

void Z_correction::set_direction() {
    if (_wrote_dir) return;
    int dir = digitalReadFast(RECIEVE_DIR);
    digitalWrite(PASS_DIR, dir);
    _wrote_dir = true;
}
