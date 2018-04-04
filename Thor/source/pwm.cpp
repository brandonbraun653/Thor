#include <Thor/include/pwm.h>

// uint32_t timerBaseAddresses[] = 
// { 
// 	#if defined(STM32F446xx) || defined(STM32F767xx)
// 	0,	/* Indexing offset since no TIM0 */
// 	TIM1_BASE,
// 	TIM2_BASE,
// 	TIM3_BASE,
// 	TIM4_BASE,
// 	TIM5_BASE,
// 	TIM6_BASE,
// 	TIM7_BASE,
// 	TIM8_BASE,
// 	TIM9_BASE,
// 	TIM10_BASE,
// 	TIM11_BASE,
// 	TIM12_BASE,
// 	TIM13_BASE,
// 	TIM14_BASE
// 	#endif
// };
// 
// PWMClass::PWMClass(int timerNumber, GPIOClass_sPtr outputPin)
// {
// 	timer_channel = timerNumber;
// 	outputPin->mode(ALT_PP, PULLDN);
// 	
// 	/*  Get timer clock source frequency */
// 	#if defined(STM32F446xx) || defined(STM32F767xx)
// // 	uint32_t periphBaseMask = 0xFFFF0000;
// // 	
// // 	LL_RCC_ClocksTypeDef currentClockConfig;
// // 	LL_RCC_GetSystemClocksFreq(&currentClockConfig);
// // 	
// // 	if ((periphBaseMask & timerBaseAddresses[timerNumber]) == APB1PERIPH_BASE) 
// // 		timerClockFrequency = 2*currentClockConfig.PCLK1_Frequency;
// // 	
// // 	if ((periphBaseMask & timerBaseAddresses[timerNumber]) == APB2PERIPH_BASE)
// // 		timerClockFrequency = 2*currentClockConfig.PCLK2_Frequency;
// 	#endif
//  }
// 
// PWMClass::~PWMClass()
// {
// 
// }
// 
// void PWMClass::initialize(float frequency, float dutyCycle)
// {
// 	
// 	volatile uint32_t temp = (0xFFFF0000 & TIM1_BASE);
// 	volatile uint32_t temp2 = (0xFFFF0000 & TIM2_BASE);
// 	temp += 1;
// }
// 
// void PWMClass::enable()
// {
// }
// 
// void PWMClass::disable()
// {
// }
// 
// void PWMClass::updateDutyCycle(float dutyCycle)
// {
// }
