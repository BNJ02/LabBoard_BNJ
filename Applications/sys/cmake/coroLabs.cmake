#directory names
set(LIB_DIR ${CMAKE_SOURCE_DIR}/../lib)
set(SYS_DIR ${CMAKE_SOURCE_DIR}/../sys)
set(CMSIS_DIR ${SYS_DIR}/CMSIS)

#tft option (lot of flash!)
if(WITH_TFT)
	set(TFT_LIB_DIR ${LIB_DIR}/tft)
	set(TFT_LIB_ARDUINO_DIR ${TFT_LIB_DIR}/arduinoCore)
	set(TFT_LIB_ADAFRUIT_ST7735_DIR ${TFT_LIB_DIR}/Adafruit-ST7735-Library)
	set(TFT_LIB_ADAFRUIT_GFX_DIR ${TFT_LIB_DIR}/Adafruit-GFX-Library)
	set(TFT_LIB_ADAFRUIT_GFX_FONT_DIR ${TFT_LIB_ADAFRUIT_GFX_DIR}/Fonts)
	set(TFT_SRCS 
		${TFT_LIB_DIR}/tft.cpp 
		${TFT_LIB_ARDUINO_DIR}/Print.cpp 
		${TFT_LIB_ADAFRUIT_GFX_DIR}/Adafruit_GFX.cpp
		${TFT_LIB_ADAFRUIT_GFX_DIR}/Adafruit_SPITFT.cpp 
		${TFT_LIB_ADAFRUIT_ST7735_DIR}/Adafruit_ST7735.cpp
		${TFT_LIB_ADAFRUIT_ST7735_DIR}/Adafruit_ST77xx.cpp
		${LIB_DIR}/spi.c
	)
	set(TFT_INCLUDES 
		${TFT_LIB_DIR}
		${TFT_LIB_ARDUINO_DIR}
		${TFT_LIB_ADAFRUIT_ST7735_DIR}
		${TFT_LIB_ADAFRUIT_GFX_DIR}
		${TFT_LIB_ADAFRUIT_GFX_FONT_DIR}
	)
endif()

#lib directories included
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${LIB_DIR}
	${TFT_INCLUDES}
)

set(LIBSRCS
    ${LIB_DIR}/adc.c
    ${LIB_DIR}/pinAccess.c
    ${LIB_DIR}/spi.c
#   ${LIB_DIR}/dummySerial.c
    ${LIB_DIR}/exti.c
)

set(APP_SRCS ${SRCS} ${LIBSRCS} ${TFT_SRCS})

#STM32F303 compiler/linker specific stuff
# => APP_SRCS and SYS_DIR should defined.
include("../sys/cmake/stm32f303.cmake")
