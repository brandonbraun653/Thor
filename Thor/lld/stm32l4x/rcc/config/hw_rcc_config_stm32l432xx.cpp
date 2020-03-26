/********************************************************************************
 *  File Name:
 *    hw_rcc_config_stm32l432xx.cpp
 *
 *  Description:
 *    RCC Configuration Options for STM32L432
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_RCC ) && defined( STM32L432xx )

#ifdef __cplusplus
extern "C"
{
#endif

  uint32_t SystemCoreClock           = 16000000;
  const uint8_t AHBPrescTable[ 16 ]  = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9 };
  const uint8_t APBPrescTable[ 8 ]   = { 0, 0, 0, 0, 1, 2, 3, 4 };
  const uint32_t MSIRangeTable[ 12 ] = { 100000U,  200000U,  400000U,   800000U,   1000000U,  2000000U,
                                         4000000U, 8000000U, 16000000U, 24000000U, 32000000U, 48000000U };

#ifdef __cplusplus
}
#endif

#endif /* TARGET_STM32L4 && THOR_DRIVER_RCC */
