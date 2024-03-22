#ifndef ADC_H
#define ADC_H

#include "bits.h"

#ifndef ADCDEF
#define ADCDEF
#endif // ADCDEF

//CR1
#define SCAN 8
#define RES 24

// CR2
#define ADON 0
#define SWSTART 30

//SR

#define ADONS 6

#define ADC_READY (ADC1->SR & (1U << ADONS))
#define ADC_WAIT_UNTIL_READY while (!ADC_READY)
#define ADC_START ADC1->CR2 |= 1;

typedef enum {
    BIT_12 = 0,
    BIT_10 = 1,
    BIT_8 = 2,
    BIT_6 = 3,
} Resolution;

#define SCAN_ON 1
#define SCAN_OFF 0

#define adc_read

ADCDEF void adc_init(uint8_t scan_mode_on, Resolution res, uint8_t channel);

#endif // ADC_H
