#ifndef TIMERS_H
#define TIMERS_H

#include "bits.h"

#ifndef TIMDEF
#define TIMDEF
#endif

// CR1 offsets
#define ARPE 7
//SMCR offsets
#define CC1IE 1
#define CC2IE 2
#define CC3IE 3
#define CC4IE 4
//CCMR1 offsets
#define CC1S 0
#define OC1FE 2
#define OC1PE 3
#define OC1M 4
#define OC1CE 7
#define CC2S 8
#define OC2FE 10
#define OC2PE 11
#define OC2M 12
#define OC2CE 15
//CCMR2 offsets
#define CC3S 0
#define OC3FE 2
#define OC3PE 3
#define OC3M 4
#define OC3CE 7
#define CC4S 8
#define OC4FE 10
#define OC4PE 11
#define OC4M 12
#define OC4CE 15
// EGR offsets
#define CC1G 1
#define CC2G 2
#define CC3G 3
#define CC4G 4
//CCER offsets
#define CC1E 0
#define CC1P 1
#define CC1NP 3
#define CC2E 4
#define CC2P 5
#define CC2NP 7
#define CC3E 8
#define CC3P 9
#define CC3NP 11
#define CC4E 12
#define CC4P 13
#define CC4NP 15

// PWM modes
#define NORMAL 6
#define INVERSE 7

// TIC and TOC
typedef enum {
    TIC = 0,
    TOC = 1,
} Mode;
#define NO_PIN -1

#define timer_configure_interrupt(timer, channel) set_bits_in_16_register(&timer->DIER, 1, 1, channel)
#define timer_configure_event(timer, event) set_bits_in_16_register(&timer->EGR, 1, 1, event)
#define timer_configure_clock_signal(timer, psc, arr)   \
    do                                                  \
    {                                                   \
        timer->PSC = (psc) - 1;                         \
        timer->ARR = (arr) - 1;                         \
    } while (0)
#define timer_set_pwm_dc_hidden(timer, channel, dc) timer->CCR##channel = (dc)
#define timer_set_pwm_dc(timer, channel, dc) timer_set_pwm_dc_hidden(timer, channel, dc)

TIMDEF void timer_set_pwm(TIM_TypeDef* timer, uint8_t channel, uint8_t pwm_mode, uint16_t pwm_dc);

#endif // TIMERS_H
