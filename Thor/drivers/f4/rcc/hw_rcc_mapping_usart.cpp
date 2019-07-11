/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping_usart.cpp
 *
 *   Description:
 *    RCC configuration maps for the USART peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>


namespace Thor::Driver::RCC
{
/*------------------------------------------------
USART Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )
  /**
   *  USART clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_usart_mapping.hpp
   */
  const std::array<ClockEnableConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ClockConfig_USART = {
    { /* USART1 */
      { reinterpret_cast<decltype( ClockEnableConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2ENR ) ),
        APB2ENR_USART1EN },
      /* USART2 */
      { reinterpret_cast<decltype( ClockEnableConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ),
        APB1ENR_USART2EN },
      /* USART3 */
      { reinterpret_cast<decltype( ClockEnableConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ),
        APB1ENR_USART3EN },
      /* USART6 */
      { reinterpret_cast<decltype( ClockEnableConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2ENR ) ),
        APB2ENR_USART6EN } }
  };

  /**
   *  USART low power clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_usart_mapping.hpp
   */
  const std::array<ClockEnableLowPowerConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ClockConfigLP_USART = {
    { /* USART1 */
      { reinterpret_cast<decltype( ClockEnableLowPowerConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2LPENR ) ),
        APB2LPENR_USART1LPEN },
      /* USART2 */
      { reinterpret_cast<decltype( ClockEnableLowPowerConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
        APB1LPENR_USART2LPEN },
      /* USART3 */
      { reinterpret_cast<decltype( ClockEnableLowPowerConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
        APB1LPENR_USART3LPEN },
      /* USART6 */
      { reinterpret_cast<decltype( ClockEnableLowPowerConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2LPENR ) ),
        APB2LPENR_USART6LPEN } }
  };

  /**
   *  USART reset register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_usart_mapping.hpp
   */
  const std::array<PeripheralResetConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ResetConfig_USART = {
    { /* USART1 */
      { reinterpret_cast<decltype( PeripheralResetConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2RSTR ) ),
        APB2RSTR_USART1RST },
      /* USART2 */
      { reinterpret_cast<decltype( PeripheralResetConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
        APB1RSTR_USART2RST },
      /* USART3 */
      { reinterpret_cast<decltype( PeripheralResetConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
        APB1RSTR_USART3RST },
      /* USART6 */
      { reinterpret_cast<decltype( PeripheralResetConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB2RSTR ) ),
        APB2RSTR_USART6RST }

    }
  };

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
}    // namespace Thor::Driver::RCC
