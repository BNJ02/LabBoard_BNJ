# TFT basics
## Hardware part
The 1.8" color TFT display is an [Adafruit product](https://www.adafruit.com/product/358). It uses the SPI interface (as the I/O expander), with an I/O to siwth between data and command mode.


| name     | STM32 pin       | description            |
|----------|-----------------|------------------------|
| TFT_CS   | PB2             | Chip select            |
| SCK      | PB3 / SPI1_SCK  | Serial clock           |
| MOSI     | PB5 / SPI1_MOSI | Master Out, Slave In   |
| MISO     | PB4 / SPI1_MISO | Master In, Slave Out   |
| D/C      | PA12            | Data / Command switch  |

The TFT circuit is also connected to the RESET circuit. The backlight is always on, and cannot be modified.
The embedded SD card reader cannot be used with this lab board.


## Software library

A driver for the TFT display is provided, based on the Adafruit libraries [Adafruit-ST7735](https://github.com/adafruit/Adafruit-ST7735-Library) and [Adafruit-GFX](https://github.com/adafruit/Adafruit-GFX-Library), originally targetting Arduino framework.

### Compilation
As the 2 libraries require a lot files/memory, the Makefile can be configured to compile these files or not:
By default, the libraries are not compiled in, you can add them with the following line into the `CMakeList.txt` cmake file:
```sh
option(WITH_TFT "add compilation rules for the TFT support" ON)
```

**NOTE** The library is written in C++, and should be called from C++ files only. Your main file should be renamed `main.cpp` for instance.

### Init

the tft is a 2-dimension tabular of pixels, where column 0 is at the left side, and line 0 is at the top of the screen. We can use it to draw graphics (lines, circles, ..) and text.

The include files are:
```c
#include "tft.h"
```
A global object `Tft` is provided (**with a uppercase!**)
The initialization of the tft works as follow:
```c++
Tft.setup()
```

### Text writing

Text writing is based on the Arduino Print() function:
```c++
Tft.setTextColor(ST7735_RED);
Tft.setTextCursor(5, 3);
Tft.print("Hello World! ");
```
The first line configure the text color. Color are defined in file `lib/tft/Adafruit-ST7735-Library/Adafruit_ST7735.h`. The color is a 16-bit value made like this:  *red* (5 bits) | *green* (6 bits) | *blue* (5 bits).

The second defines the position to write (column, line).

The third line just print the string. As in Arduino, a value can be printed too (C++ function redefinition):
```c++
Tft.print(32,HEX); //print a value in hexadecimal
```
The definition is in file `lib/tft/arduinoCore/Print.h`

**Note** When you rewrite to an existing text section, the previous one is not removed. It can be done by filling a rectangle before printing some text:

```c++
Tft.setTextCursor(5, 3);
Tft.eraseText(6);
Tft.print("Hello!");
```

### Graphical use

The graphical functions are defined in `../lib/tft/Adafruit-GFX-Library/Adafruit_GFX.h`. **NOTE** For low-level functions, you have to start and end with `startWrite()` and `endWrite()`.

example with low level functions:
```c++
	tft.startWrite();
	tft.writeFastVLine(10,tft.height()-22,10,0);
	tft.endWrite();
```

and high level functions:
```c++
	tft.fillRect(	tft.width()/15,
			tft.height()-10,
			tft.width()/15,
			10,
			ST7735_GREEN);
```

### Full example

A full simple example (only with text) is defined in the source of lab3.
