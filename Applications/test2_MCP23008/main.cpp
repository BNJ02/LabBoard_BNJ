#include "stm32f3xx.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ST7735.h"
#include "tft.h"
#include "i2c.h"
#include "spi.h"

void setup() {
    Tft.setup();
    setupI2C();
}

int main(void) {
    setup();

    Tft.setTextCursor(5,0);
    Tft.print(RCC->APB1ENR); 

    Tft.setTextCursor(5,1);
    Tft.print(I2C1->CR2);

    Tft.setTextCursor(5,2);
    Tft.print(I2C1->CR1);

    Tft.setTextCursor(5,3);
    Tft.print(RCC->APB1RSTR);

    Tft.setTextCursor(5,4);
    Tft.print(RCC->AHBENR);

    Tft.setTextCursor(5,6);
    Tft.print(GPIOB->MODER);

    /* Infinite loop */
    while (1) 
    {
        
    }
}

