#ifndef L298N_H
#define L298N_H

#include "bits.h"

#ifndef DRIVERDEF
#define DRIVERDEF
#endif // DRIVERDEF

typedef struct {
    volatile uint32_t *in1;
    volatile uint32_t *in2;
    uint8_t in1_offset;
    uint8_t in2_offset;
} Motor;

typedef struct {
    uint32_t value_1;
    uint32_t value_2;
} Motor_State;

DRIVERDEF void driver_set_motor(Motor *motor, Motor_State* state);

#endif // L298N_H
