#include "timers.h"

// TODO: get rid of the switch-case with macros
void timer_set_pwm(TIM_TypeDef* timer, uint8_t channel, uint8_t pwm_mode, uint16_t pwm_dc)
{
    volatile uint32_t *ccmr = NULL,
        *ccr = NULL;
    uint8_t pe_offset = 0,
        m_offset = 0,
        e_offset = 0;
    switch (channel)
    {
    case 1:
        pe_offset = OC1PE;
        m_offset = OC1M;
        e_offset = CC1E;
        ccmr = &timer->CCMR1;
        ccr = &timer->CCR1;
        break;
    case 2:
        pe_offset = OC2PE;
        m_offset = OC2M;
        e_offset = CC2E;
        ccmr = &timer->CCMR1;
        ccr = &timer->CCR2;
        break;
    case 3:
        pe_offset = OC3PE;
        m_offset = OC3M;
        e_offset = CC3E;
        ccmr = &timer->CCMR2;
        ccr = &timer->CCR3;
        break;
    case 4:
        pe_offset = OC4PE;
        m_offset = OC4M;
        e_offset = CC4E;
        ccmr = &timer->CCMR2;
        ccr = &timer->CCR4;
        break;
    default:
        return;
    }
    timer->CR1 |= (1 << ARPE);
    *ccmr |= (1 << pe_offset);
    *ccr = pwm_dc;
    timer->CCER |= (1 << e_offset);
    set_bits_in_32_register(ccmr, pwm_mode, 3, m_offset);
}

void timer_set_toc(TIM_TypeDef* timer, uint8_t channel, uint32_t comp_value, uint8_t with_irq)
{
    volatile uint32_t * ccr = NULL;
    uint8_t ccye = 0;
    switch (channel)
    {
    case 1:
        ccr = &timer->CCR1;
        ccye = CC1E;
        break;
    case 2:
        ccr = &timer->CCR2;
        ccye = CC2E;
        break;
    case 3:
        ccr = &timer->CCR3;
        ccye = CC3E;
        break;
    case 4:
        ccr = &timer->CCR4;
        ccye = CC4E;
        break;
    }
    *ccr = comp_value;
    if (with_irq) timer->DIER |= (1 << ccye);
}
