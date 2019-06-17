
#include <Thor/headers.hpp>
#include <Thor/macro.hpp>
#include <Thor/core.hpp>

#if !defined( USING_FREERTOS )
void SysTick_Handler( void )
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
#endif

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
/* Sets the clock to 216 MHz and maxes out the peripheral clocks */
#if defined( STM32F767xx )
void ThorSystemClockConfig()
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 16;
  RCC_OscInitStruct.PLL.PLLN            = 432;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  /**Activate the Over-Drive mode
   */
  if ( HAL_PWREx_EnableOverDrive() != HAL_OK )
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_7 ) != HAL_OK )
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq() / 1000 );

  /**Configure the Systick
   */
  HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}
#endif

/* Sets the clock to 180 MHz and maxes out the peripheral clocks */
#if defined( STM32F446xx )
void ThorSystemClockConfig()
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI; /* Internal 16MHz clock */
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;

  RCC_OscInitStruct.HSEState = RCC_HSE_OFF; /* External high speed osc off */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF; /* Internal low speed osc off */
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;  /* Using external RC clock oscillator*/

  RCC_OscInitStruct.PLL.PLLState  = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM      = 8;
  RCC_OscInitStruct.PLL.PLLN      = 128;
  RCC_OscInitStruct.PLL.PLLP      = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ      = 2;
  RCC_OscInitStruct.PLL.PLLR      = 2;

  assert( HAL_RCC_OscConfig( &RCC_OscInitStruct ) == HAL_OK );

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  assert( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_5 ) == HAL_OK );

  /* Configure the Systick interrupt time */
  HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq() / 1000 );

  /* Configure the Systick */
  HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}
#endif
#endif 