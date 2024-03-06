#ifndef GPIO_H
#define GPIO_H

#define BITS_IMPLEMENTATION
#include "./bits.h"

#ifndef GPIODEF
#define GPIODEF
#endif

// Modes for MODER
#define INPUT 0
#define OUTPUT 1
#define AF 2
#define ANALOG 3

// Modes for PUPDR
#define FLOATING 0
#define PULLUP 1
#define PULLDOWN 2

// Modes for OTYPER
#define PUSHPULL 0
#define ODRAIN 1

// Alternate functions
#define AF0 0
#define AF1 1
#define AF2 2
#define AF3 3
#define AF4 4
#define AF5 5
#define AF6 6
#define AF7 7
#define AF8 8
#define AF9 9
#define AF10 10
#define AF11 11
#define AF12 12
#define AF13 13
#define AF14 14
#define AF15 15

#define gpio_set_pin_mode(gpio, port, value) set_bits_in_32_register(&gpio->MODER, value, 2, 2 * port);
#define gpio_set_pin_pupdr(gpio, port, value) set_bits_in_32_register(&gpio->PUPDR, value, 2, 2 * port);
#define gpio_set_pin_type(gpio, port, value) set_bits_in_32_register(&gpio->OTYPER, value, 1, port);
GPIODEF void gpio_set_pin_alternate_function(GPIO_TypeDef* gpio, uint8_t port, uint8_t value);

#define gpio_set_pin(gpio, port) gpio->BSRR |= (1 << port);
#define gpio_reset_pin(gpio, port) gpio->BSRR |= (1 << (port + 16));
#define gpio_toggle_pin(gpio, port) gpio->ODR ^= (1 << port);

#endif //GPIO_H

/* --------END OF HEADER -------- */

#ifdef GPIO_IMPLEMENTATION

void gpio_set_pin_alternate_function(GPIO_TypeDef *gpio, uint8_t port, uint8_t value)
{
    uint32_t* reg = NULL;
    if (port >= 16)
        return;
    if (port >= 7)
        reg = gpio->AFR[1];
    else
        reg = gpio->AFR[0];
    set_bits_in_32_register(reg, value, 4, 4 * port);
}
#endif // GPIO_IMPLEMENTATION
