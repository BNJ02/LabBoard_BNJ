#include "stm32f3xx.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ST7735.h"
#include "tft.h"
#include "i2c.h"
#include "spi.h"

void setup() {
    Tft.setup();
    setupI2C();
    //initI2C();
    //Tft.setup();
}

int main(void) {
    setup();

    //Tft.setTextCursor(5,0);
    //Tft.print("Hello world !");

    I2C1->CR2     &= ~(I2C_CR2_SADD);
    I2C1->CR2     |=  (0x00 << I2C_CR2_SADD_Pos);

    // Write two bytes; the register offset, and its value.
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);
    I2C1->CR2 |=  (0x01 << I2C_CR2_NBYTES_Pos);

    //I2C1->CR1 |= 0x000000FF;

    //writeI2C(0x20, 0x01, 1);

    Tft.setTextCursor(5,1);
    Tft.print(I2C1->CR2);

    Tft.setTextCursor(5,2);
    Tft.print(I2C1->CR1);

    /* Infinite loop */
    while (1) 
    {
        writeI2C(0x20, 0x01, 1);
    }
}

