#include "gpio.h"

void gpio_set_pin_alternate_function(GPIO_TypeDef *gpio, uint8_t port, uint8_t value)
{
    volatile uint32_t* reg = NULL;
    if (port >= 16)
        return;
    if (port >= 8)
    {
    	port -= 8;
        reg = &gpio->AFR[1];
    }
    else
        reg = &gpio->AFR[0];
    set_bits_in_32_register(reg, value, 4, 4 * port);
}
