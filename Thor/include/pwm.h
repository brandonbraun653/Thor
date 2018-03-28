#pragma once
#ifndef PWM_H_
#define PWM_H_

#include "thor_config.h"
#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_rcc.h>
#endif

#include "thor_definitions.h"
#include "gpio.h"


class PWMClass
{
public:
	void initialize(float frequency, float dutyCycle);
	void enable();
	void disable();
	void updateDutyCycle(float dutyCycle);

	/* Current config reporting */
	struct PWMSettings_t
	{
		float frequency;	/* Hz  */
		float dutyCycle;	/* %   */
		float onTime;		/* sec */
		float offTime;		/* sec */
	} PWMOutputState;

	
	PWMClass(int timerNumber, GPIOClass_sPtr outputPin);
	~PWMClass();

private:
	
	
	int timer_channel;
	uint32_t timerClockFrequency;
};

#endif /*! PWM_H_ */