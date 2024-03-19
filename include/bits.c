#include "bits.h"

void set_bits_in_register(volatile void *reg, uint8_t reg_size, uint32_t value, uint8_t size, uint8_t offset)
{
    if (size > reg_size ||
        offset > reg_size - 1 ||
        size + offset > reg_size)
        return;

    uint32_t mask = (1 << size) - 1;
    content &= mask;
    uint32_t neg = ~content & mask;

    *reg |= (content << offset);
    *reg &= ~(neg << offset);
    
    //for(uint8_t count = 0; count < size; ++count)
    //{
    //    uint8_t x = (value >> count) & 1;
    //    if (x)
    //        *(uintptr_t *) reg |= (1 << (offset + count));
    //    else
    //        *(uintptr_t *) reg &= ~(1 << (offset + count));
    //}
}

void set_bits_in_32_register(volatile uint32_t* reg, uint32_t value, uint8_t size, uint8_t offset)
{
    set_bits_in_register(reg, 32, value, size, offset);
}

void set_bits_in_16_register(volatile uint16_t* reg, uint32_t value, uint8_t size, uint8_t offset)
{
    set_bits_in_register(reg, 16, value, size, offset);
}

void set_bits_in_8_register(volatile uint8_t* reg, uint32_t value, uint8_t size, uint8_t offset)
{
    set_bits_in_register(reg, 8, value, size, offset);
}
