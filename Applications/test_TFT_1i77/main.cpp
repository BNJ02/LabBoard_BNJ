#include "stm32f3xx.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ST7735.h"
#include "tft.h"

void setup() {
    Tft.setup();
    Tft.setTextCursor(0,5);
    Tft.print("Coucou Franck !");
}

int main(void) {
    setup();

    /* Infinite loop */
    while (1) {

    }
}

