//
// Created by liav on 29/05/17.
//

#ifndef STEPPER_CORRECTIONS_H
#define STEPPER_CORRECTIONS_H

#include <Arduino.h>

#define ENABLE LOW
#define DISABLE HIGH
#define DELAY_TIME 10
#define REPEAT(NUM_STEPS) for(int i = NUM_STEPS; i--;)
#define FORCE_INLINE __attribute__((always_inline)) inline

class Z_correction {

public:
    Z_correction(int Z_enable_pin, int Z_step_pin, int Z_dir_pin);
    FORCE_INLINE void enable();
    FORCE_INLINE void disable();
    FORCE_INLINE void apply_steps(int num_of_steps);
    FORCE_INLINE void set_direction(bool clockwise);

private:
    int _Z_enable_pin;
    int _Z_step_pin;
    int _Z_dir_pin;
};



#endif //STEPPER_CORRECTIONS_H
