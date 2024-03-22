#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include "bits.h"

#ifndef INTDEF
#define INTDEF
#endif // INTDEF

#define EXTI0 0
#define EXTI1 1
#define EXTI2 2
#define EXTI3 3
#define EXTI4 4
#define EXTI5 5
#define EXTI6 6
#define EXTI7 7
#define EXTI8 8
#define EXTI9 9
#define EXTI10 10
#define EXTI11 11
#define EXTI12 12
#define EXTI13 13
#define EXTI14 14
#define EXTI15 15

// NVIC interrupt numbers
#define NVIC_EXTI0 6
#define NVIC_EXTI1 7
#define NVIC_EXTI2 8
#define NVIC_EXTI3 9
#define NVIC_EXTI9_5 23
#define NVIC_EXTI10_15 40
#define NVIC_TIM2 28
#define NVIC_TIM3 29
#define NVIC_TIM4 30

typedef enum {
    RISING_EDGE = 1,
    FALLING_EDGE = 2,
    BOTH_EDGES = 3,
} RF_Mode;

typedef enum {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    H = 4,
    E = 5,
} Selected_GPIO;

INTDEF void configure_interrupt(uint8_t int_number, RF_Mode rf_mode, Selected_GPIO gpio);

static inline void unmask_interrupt(uint8_t int_number)
{
    uint8_t nvic_subarray = (int_number >= 32) ? 1 : 0;
    NVIC->ISER[nvic_subarray] |= (1 << (int_number % 32));
}

static inline void mask_interrupt(uint8_t int_number)
{
    uint8_t nvic_subarray = (int_number >= 32) ? 1 : 0;
    NVIC->ICER[nvic_subarray] |= (1 << (int_number % 32));
}

#endif // INTERRUPTS_H
