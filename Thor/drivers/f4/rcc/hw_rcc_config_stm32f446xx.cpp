/********************************************************************************
 *   File Name:
 *    hw_rcc_config_stm32f446xx.cpp
 *
 *   Description:
 *    Contains implementation details for reset and clock configuration options
 *    for the STM32F446xx.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_config_stm32f446xx.hpp>

#ifdef  __cplusplus
extern "C"
{
#endif

  uint32_t SystemCoreClock = 16000000;

#ifdef  __cplusplus
}
#endif