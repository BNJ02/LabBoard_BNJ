#ifndef __EXTI_H__
#define __EXTI_H__

#include "stm32f3xx.h"

#ifdef __cplusplus
  extern "C" {
#endif /* __cplusplus */

//mode (mask)
#define RISING  0x1
#define FALLING 0x2
#define CHANGE  0x3 /* both rising and falling */
#define BOTH    0x3 /* both rising and falling */

/* configure a pin
 * - port is GPIOA, GPIOB, ...
 * - numBit is the pin number (0 to 15)
 * - mode is in RISING, FALLING or CHANGE
 */
int attachInterrupt(GPIO_TypeDef *port, 
                    unsigned char numBit,
                    unsigned char mode);


#ifdef __cplusplus
  }
#endif /* __cpusplus */

#endif
