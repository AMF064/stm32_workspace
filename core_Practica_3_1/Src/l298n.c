#include "l298n.h"

void driver_set_motor(Motor *motor, Motor_State * state)
{
    set_bits_in_32_register(motor->in1, state->value_1, 32 - motor->in1_offset, motor->in1_offset);
    set_bits_in_32_register(motor->in2, state->value_2, 32 - motor->in2_offset, motor->in2_offset);
}
