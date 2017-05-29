//
// Created by liav on 29/05/17.
//

#ifndef STEPPER_CORRECTIONS_H
#define STEPPER_CORRECTIONS_H

#include <Arduino.h>

#define ENABLE LOW
#define DISABLE HIGH
#define DELAY_TIME 300
#define REPEAT(NUM_STEPS) for(int i = NUM_STEPS; i--;)

class Z_correction {

public:
    Z_correction(int Z_enable_pin, int Z_step_pin, int Z_dir_pin);
    void apply_steps(int num_of_steps, bool clockwise);

private:
    int _Z_enable_pin;
    int _Z_step_pin;
    int _Z_dir_pin;
};



#endif //STEPPER_CORRECTIONS_H
