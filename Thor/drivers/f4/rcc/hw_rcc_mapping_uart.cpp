/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping_uart.cpp
 *
 *   Description:
 *    RCC configuration maps for the UART peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>

namespace Thor::Driver::RCC::LookupTables
{
/*------------------------------------------------
UART Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )
  /**
   *  UART clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  const ClockEnableConfig ClockConfig_UART[ uartTableSize ] = {
    /* UART4 */
    { reinterpret_cast<decltype( ClockEnableConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ),
      APB1ENR_UART4EN },
    /* UART5 */
    { reinterpret_cast<decltype( ClockEnableConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ),
      APB1ENR_UART5EN }
  };

  /**
   *  UART low power clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  const ClockEnableLowPowerConfig ClockConfigLP_UART[ uartTableSize ] = {
    /* UART4 */
    { reinterpret_cast<decltype( ClockEnableLowPowerConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
      APB1LPENR_UART4LPEN },
    /* UART5 */
    { reinterpret_cast<decltype( ClockEnableLowPowerConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
      APB1LPENR_UART5LPEN }
  };

  /**
   *  UART reset register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  const PeripheralResetConfig ResetConfig_UART[ uartTableSize ] = {
    /* UART4 */
    { reinterpret_cast<decltype( PeripheralResetConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
      APB1RSTR_UART4RST },
    /* UART5 */
    { reinterpret_cast<decltype( PeripheralResetConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
      APB1RSTR_UART5RST }
  };

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
}    // namespace Thor::Driver::RCC::LookupTables
