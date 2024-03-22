#include "adc.h"

void adc_init(uint8_t scan_mode_on, Resolution res, uint8_t channel)
{
    ADC1->CR2 &= ~0x1;
    if (scan_mode_on) ADC1->CR1 |= (1 << SCAN);
    set_bits_in_32_register(&ADC1->CR1, res, 2, RES);
    ADC1->SMPR1 = ADC1->SMPR2 = ADC1->SMPR3 = 0;
    ADC1->SQR1 = 0;
    set_bits_in_32_register(&ADC1->SQR5, channel, 4, 0);
}
