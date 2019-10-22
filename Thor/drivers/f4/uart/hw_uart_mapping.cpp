/********************************************************************************
 *   File Name:
 *    hw_uart_mapping.cpp
 *
 *   Description:
 *    Useful maps for the UART peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/uart/hw_uart_mapping.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )

namespace Thor::Driver::UART
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;
  DMASignalList RXDMASignals;
  DMASignalList TXDMASignals;
  DriverInstanceList uartObjects;

  const IRQn_Type UART_IRQn[ NUM_UART_PERIPHS ] = { UART4_IRQn, UART5_IRQn };


  void initializeMapping()
  {
    uartObjects.fill( nullptr );
  }

  bool isUART( const std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
      }
    }
    return result;
  }
}    // namespace Thor::Driver::UART

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
