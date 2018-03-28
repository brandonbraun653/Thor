#pragma once
#ifndef TIMER_H_
#define TIMER_H_

#include "thor.h"
#include "thor_config.h"
#include "thor_definitions.h"

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif


extern void TIMER_EnableClock(int timerNumber);
extern void TIMER_DisableClock(int timerNumber);

#endif