#ifndef BITS_H
#define BITS_H

#ifndef BITSDEF
#define BITSDEF
#endif

#include <stdint.h>
#include "stm32l1xx_hal.h"

BITSDEF void set_bits_in_register(volatile void *reg, uint8_t reg_size, uint32_t value, uint8_t size, uint8_t offset);
BITSDEF void set_bits_in_32_register(volatile uint32_t* reg, uint32_t value, uint8_t size, uint8_t offset);
BITSDEF void set_bits_in_16_register(volatile uint16_t* reg, uint32_t value, uint8_t size, uint8_t offset);
BITSDEF void set_bits_in_8_register(volatile uint8_t* reg, uint32_t value, uint8_t size, uint8_t offset);

#endif // BITS_H
