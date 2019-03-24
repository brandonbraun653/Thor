#pragma once
#ifndef PWM_H_
#define PWM_H_

#include <Thor/config.hpp>
#include <Thor/definitions.hpp>
#include <Thor/gpio.hpp>

#ifdef TARGET_STM32F7

#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_ll_rcc.h>
#endif

namespace Thor
{
  // TODO: Not even remotely finished
  // 	class PWMClass
  // 	{
  // 	public:
  // 		void initialize(float frequency, float dutyCycle);
  // 		void enable();
  // 		void disable();
  // 		void updateDutyCycle(float dutyCycle);
  //
  // 		/* Current config reporting */
  // 		struct PWMSettings_t
  // 		{
  // 			float frequency; /* Hz  */
  // 			float dutyCycle; /* %   */
  // 			float onTime; /* sec */
  // 			float offTime; /* sec */
  // 		} PWMOutputState;
  //
  //
  // 		PWMClass(int timerNumber, GPIOClass_sPtr outputPin);
  // 		~PWMClass();
  //
  // 	private:
  // 		int timer_channel;
  // 		uint32_t timerClockFrequency;
  // 	};
}    // namespace Thor


#endif /*! PWM_H_ */
