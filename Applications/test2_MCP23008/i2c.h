#ifndef __I2C_H__
#define __I2C_H__
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


void setupI2C();

void startI2C(uint8_t slave_address, uint8_t RW, uint8_t nb_bytes);

void writeI2C(uint8_t slave_address, uint8_t data, uint8_t nb_bytes);

uint8_t readI2C(uint8_t slave_address, uint8_t nb_bytes);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

