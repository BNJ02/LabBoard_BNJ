#include "stm32f3xx.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ST7735.h"
#include "tft.h"
#include "i2c.h"
#include "spi.h"

/** affichage d'une valeur 
 *  au format virgule fixe 10.6
 */
void affVF10_6(int16_t val)
{
	//seul endroit avec des 'float' car c'est du debug!!
	Tft.print(((float)(val))/64);
}

void setup() {
    Tft.setup();
}

int main(void) {
    setup();

    Tft.setTextCursor(5,0);
    Tft.print("Coucou Leila !");

    /* Infinite loop */
    while (1) {

    }
}

