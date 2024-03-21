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

#define ADC_READY (ADC1->SR & (1U << ADONS))
#define ADC_WAIT_UNTIL_READY while (!ADC_READY)


typedef enum {
    12_BIT = 0,
    10_BIT = 1,
    8_BIT = 2,
    6_BIT = 3,
} Resolution;

#define SCAN_ON 1
#define SCAN_OFF 0

ADCDEF void adc_start(uint8_t scan_mode_on, Resolution res, uint8_t channel);

#endif // ADC_H
