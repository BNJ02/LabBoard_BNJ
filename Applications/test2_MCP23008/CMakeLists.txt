#to compile the project:
# mkdir _build
# cd _build
# cmake -D CMAKE_TOOLCHAIN_FILE=../../sys/cmake/arm-none-eabi.cmake ..
# make
# make flash
cmake_minimum_required(VERSION 3.5)

set(SYS_DIR ${CMAKE_SOURCE_DIR}/../sys)
include(${SYS_DIR}/cmake/arm-none-eabi.cmake)

#with that, cmake will not try to compile a simple test program
# (that may fail for an embedded target)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

#project name
project(test_MCP23008)

#application sources
set(SRCS
	${CMAKE_SOURCE_DIR}/main.cpp
	${CMAKE_SOURCE_DIR}/i2c.c
)

option(WITH_TFT "add compilation rules for the TFT support" ON)
include("../sys/cmake/coroLabs.cmake")
