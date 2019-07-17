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
#include <Thor/drivers/f4/uart/hw_uart_mapping.hpp>

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
  const RegisterConfig UART_ClockConfig[ uartTableSize ] = {
    /* UART4 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ),
      APB1ENR_UART4EN },
    /* UART5 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ),
      APB1ENR_UART5EN }
  };

  /**
   *  UART low power clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  const RegisterConfig UART_ClockConfigLP[ uartTableSize ] = {
    /* UART4 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
      APB1LPENR_UART4LPEN },
    /* UART5 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
      APB1LPENR_UART5LPEN }
  };

  /**
   *  UART reset register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  const RegisterConfig UART_ResetConfig[ uartTableSize ] = {
    /* UART4 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
      APB1RSTR_UART4RST },
    /* UART5 */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
      APB1RSTR_UART5RST }
  };

  /**
   *  UART clocking bus source identifier
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  const Configuration::ClockType_t UART_SourceClock[ uartTableSize ] = {
    /* UART 4 */
    Configuration::ClockType::PCLK1,
    /* UART 5*/
    Configuration::ClockType::PCLK1
  };

  const PCC UARTLookup = { UART_ClockConfig, UART_ClockConfigLP, UART_ResetConfig, UART_SourceClock, &Thor::Driver::UART::InstanceToResourceIndex, uartTableSize };

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
}    // namespace Thor::Driver::RCC::LookupTables
