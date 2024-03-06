#ifndef TIMERS_H
#define TIMERS_H

#define BITS_IMPLEMENTATIONS
#include "./bits.h"

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
#define CC3S 8
#define OC3FE 10
#define OC3PE 11
#define OC3M 12
#define OC3CE 15
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

#define timer_configure_interrupt(timer, channel) set_bits_in_16_register(&timer->DIER, 1, 1, channel)
#define timer_configure_event(timer, event) set_bits_in_16_register(&timer->EGR, 1, 1, event)
#define timer_configure_clock_signal(timer, psc, arr)   \
    do                                                  \
    {                                                   \
        timer->PSC = (psc) - 1;                         \
        timer->ARR = (arr) - 1;                         \
    } while (0)
#define timer_set_pwm_dc(timer, channel, dc) timer->CCR(channel) = (dc)

TIMDEF void timer_set_pwm_mode(TIM_TypeDef* timer, uint8_t channel, uint8_t pwm_mode);

#endif // TIMERS_H

/* -------- END OF HEADER --------*/

#ifdef TIMERS_IMPLEMENTATION

void timer_set_pwm_mode(TIM_TypeDef* timer, uint8_t channel, uint8_t pwm_mode)
{
    uint16_t *ccmr = NULL;
    uint8_t pe_offset = 0,
        m_offset = 0,
        e_offset = 0;
    switch (channel)
    {
    case 1:
        pe_offset = OC1PE;
        m_offset = OC1M;
        e_offset = CC1E;
        ccmr = timer->CCMR1;
        break;
    case 2:
        pe_offset = OC2PE;
        m_offset = OC2M;
        e_offset = CC2E;
        ccmr = timer->CCMR1;
        break;
    case 3:
        pe_offset = OC3PE;
        m_offset = OC3M;
        e_offset = CC3E;
        ccmr = timer->CCMR2;
        break;
    case 4:
        pe_offset = OC4PE;
        m_offset = OC4M;
        e_offset = CC4E;
        ccmr = timer->CCMR2;
        break;
    default:
        return;
    }
    timer->CR1 |= (1 << ARPE);
    timer->EGR |= 1;
    *ccmr |= (1 << pe_offset);
    timer->CCER |= (1 << e_offset);
    set_bits_in_16_register(ccmr, pwm_mode, 3, m_offset);
}

#endif // TIMERS_IMPLEMENTATION
