#ifndef SERIAL_H_
#define SERIAL_H_

/************************************************************************/
/*							   Includes                                 */
/************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "thor_config.h"
#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif
#include "thor_definitions.h"
#include "uart.h"


#endif /* !SERIAL_H_ */