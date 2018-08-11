#include <Thor/include/timer.hpp>



void TIMER_EnableClock(int timerNumber)
{
	switch (timerNumber)
	{
	#if defined(STM32F446xx) || defined(STM32F767xx)
	case 1:
		__HAL_RCC_TIM1_CLK_ENABLE();
		break;
		
	case 2:
		__HAL_RCC_TIM2_CLK_ENABLE();
		break;
		
	case 3:
		__HAL_RCC_TIM3_CLK_ENABLE();
		break;
		
	case 4:
		__HAL_RCC_TIM4_CLK_ENABLE();
		break;
		
	case 5:
		__HAL_RCC_TIM5_CLK_ENABLE();
		break;
		
	case 6:
		__HAL_RCC_TIM6_CLK_ENABLE();
		break;
		
	case 7:
		__HAL_RCC_TIM7_CLK_ENABLE();
		break;
		
	case 8:
		__HAL_RCC_TIM8_CLK_ENABLE();
		break;
		
	case 9:
		__HAL_RCC_TIM9_CLK_ENABLE();
		break;
		
	case 10:
		__HAL_RCC_TIM10_CLK_ENABLE();
		break;
		
	case 11:
		__HAL_RCC_TIM11_CLK_ENABLE();
		break;
		
	case 12:
		__HAL_RCC_TIM12_CLK_ENABLE();
		break;
		
	case 13:
		__HAL_RCC_TIM13_CLK_ENABLE();
		break;
		
	case 14:
		__HAL_RCC_TIM14_CLK_ENABLE();
		break;
	#endif
		
	default: break;
	}
}

void TIMER_DisableClock(int timerNumber)
{
	switch (timerNumber)
	{
	#if defined(STM32F446xx) || defined(STM32F767xx)
	case 1:
		__HAL_RCC_TIM1_CLK_DISABLE();
		break;
		
	case 2:
		__HAL_RCC_TIM2_CLK_DISABLE();
		break;
		
	case 3:
		__HAL_RCC_TIM3_CLK_DISABLE();
		break;
		
	case 4:
		__HAL_RCC_TIM4_CLK_DISABLE();
		break;
		
	case 5:
		__HAL_RCC_TIM5_CLK_DISABLE();
		break;
		
	case 6:
		__HAL_RCC_TIM6_CLK_DISABLE();
		break;
		
	case 7:
		__HAL_RCC_TIM7_CLK_DISABLE();
		break;
		
	case 8:
		__HAL_RCC_TIM8_CLK_DISABLE();
		break;
		
	case 9:
		__HAL_RCC_TIM9_CLK_DISABLE();
		break;
		
	case 10:
		__HAL_RCC_TIM10_CLK_DISABLE();
		break;
		
	case 11:
		__HAL_RCC_TIM11_CLK_DISABLE();
		break;
		
	case 12:
		__HAL_RCC_TIM12_CLK_DISABLE();
		break;
		
	case 13:
		__HAL_RCC_TIM13_CLK_DISABLE();
		break;
		
	case 14:
		__HAL_RCC_TIM14_CLK_DISABLE();
		break;
	#endif
		
	default: break;
	}
}