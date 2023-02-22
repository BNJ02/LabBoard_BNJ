# define CPU OPTIONS
#set(CPU_OPTIONS -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F303x8 -DARMCM4)
set(CPU_OPTIONS -mthumb -mcpu=cortex-m4 -mfloat-abi=soft -DSTM32F303x8 -DARMCM4)

set(CMAKE_C_FLAGS_DEBUG   "-O0 -Wno-deprecated -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wno-deprecated -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE   "-O3") #remove the -Werror => pb with Adafruit lib.
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#compiler options
add_compile_options(
    ${CPU_OPTIONS}
	$<$<COMPILE_LANGUAGE:C>:-std=c11>
	$<$<COMPILE_LANGUAGE:CXX>:-std=c++11>          #CMSIS do not support C++17
    $<$<COMPILE_LANGUAGE:CXX>:-fms-extensions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
	#$<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>   #Adafruit
	#$<$<COMPILE_LANGUAGE:CXX>:-Wsuggest-override> #GFX does not support it
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    -fstrict-volatile-bitfields
    -ffunction-sections
    -Wall
    -Wextra
    -Wcast-align
	#-Wconversion
    #-Wsign-conversion
    -Wshadow
    -Wlogical-op
    -Wsuggest-final-types
    -Wsuggest-final-methods
    #-pedantic
    -fexceptions
    -g
)

include_directories(
    ${CMSIS_DIR}/Device/ST/STM32F3xx/Include
    ${CMSIS_DIR}/Include
)

set(SYSTEMSRCS
    ${SYS_DIR}/startup_ARMCM4.c
    ${SYS_DIR}/startup_clock.c
)

#executable (should be after compile options)
add_executable(${PROJECT_NAME} ${APP_SRCS} ${SYSTEMSRCS})

# select linker script
set(LINKER_DIR ${SYS_DIR})
set(LINKER_SCRIPT "stm32f303K8.ld")
set(LINKER_FLAGS 
	-Wl,--start-group -lm  -Wl,--end-group -Wl,--gc-sections
	-Lsys
	-lstdc++_nano
	-Xlinker -Map=${PROJECT_NAME}.map
)

#should be defined after 'add_executable'
target_link_libraries(${PROJECT_NAME}
	${CPU_OPTIONS}
	-T${LINKER_DIR}/${LINKER_SCRIPT}
	${LINKER_FLAGS}
    m
    #-nostartfiles
    #-nostdlib
)

set_property(TARGET ${PROJECT_NAME} PROPERTY LINK_DEPENDS ${LINKER_DIR}/${LINKER_SCRIPT})

#flashing rules
include("../sys/cmake/flash.cmake") 
