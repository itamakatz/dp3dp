//
// Created by liav on 29/05/17.
//

#include "stepper_corrections.h"


Z_correction::Z_correction(int Z_enable_pin, int Z_step_pin, int Z_dir_pin)
    : _Z_enable_pin(Z_enable_pin),
      _Z_step_pin(Z_step_pin),
      _Z_dir_pin(Z_dir_pin)
{}


void Z_correction::enable() {
    digitalWrite(_Z_enable_pin, ENABLE);
}

void Z_correction::disable() {
    digitalWrite(_Z_enable_pin, DISABLE);
}


void Z_correction::apply_steps(int num_of_steps) {
    REPEAT(num_of_steps){
        digitalWrite(_Z_step_pin, LOW);
        digitalWrite(_Z_step_pin, HIGH);
        delayMicroseconds(DELAY_TIME);
    }
}

void Z_correction::set_direction(bool clockwise) {
    digitalWrite(_Z_dir_pin, clockwise ? HIGH: LOW);
}