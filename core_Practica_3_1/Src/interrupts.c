#include "interrupts.h"

void configure_interrupt(uint8_t int_number, RF_Mode rf_mode)
{
    uint8_t int_num_in_exticr = int_number % 4;
    EXTI->IMR |= (1 << int_number);
    if (rf_mode & 1) EXTI->RTSR |= (1 << int_number);
    if (rf_mode & 2) EXTI->FTSR |= (1 << int_number);
    set_bits_in_32_register(&SYSCFG->EXTICR[int_number / 4], sel_gpio, 3, int_num_in_exticr);
    if (EXTI0 <= int_number && int_number <= EXTI4)
    {
        NVIC->ISER[0] |= (1 << (int_number + 6));
    }
    else if (EXTI5 <= int_number && int_number <= EXTI9)
    {
        NVIC->ISER[0] |= (1 << 23);
    }
    else if (EXTI10 <= int_number && int_number <= EXTI15_15)
    {
        NVIC->ISER[1] |= (1 << 40);
    }
}
