#ifndef THOR_H_
#define THOR_H_

#include "thor_config.h"

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif


#include "thor_definitions.h"
#include "mem.h"
#include "exceptions.h"
#include "defaults.h"
#include "core.h"
#include "serial.h"
#include "gpio.h"
#include "interrupt.h"
#include "print.h"


extern void ThorInit();

#endif /* THOR_H_ */
