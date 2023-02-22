#include "I2C.h"
#include "stm32f3xx.h"

// use I2C:
// SCL : SCL
// SDA : SDA

void setupSPI() {
    // 1 - input clock = 64MHz.
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    __asm__("nop");
    // reset peripheral (mandatory!)
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    // init procedure p.816
}