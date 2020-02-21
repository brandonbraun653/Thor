/********************************************************************************
 *   File Name:
 *    hw_usart_mapping.cpp
 *
 *   Description:
 *    Useful maps for the USART peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/interrupt/hw_it_prj.hpp>
#include <Thor/drivers/f4/usart/hw_usart_mapping.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;
  DMASignalList RXDMASignals;
  DMASignalList TXDMASignals;
  DriverInstanceList usartObjects;


  const IRQn_Type USART_IRQn[ NUM_USART_PERIPHS ] = { USART1_IRQn, USART2_IRQn, USART3_IRQn, USART6_IRQn };

  const std::array<uint32_t, NUM_USART_PERIPHS> CharWidToRegConfig = { Configuration::WordLength::LEN_8BIT,
                                                                       Configuration::WordLength::LEN_9BIT };

  const std::array<uint32_t, NUM_USART_PERIPHS> ParityToRegConfig = { Configuration::Parity::NONE, Configuration::Parity::EVEN,
                                                                      Configuration::Parity::ODD };

  const std::array<uint32_t, NUM_USART_PERIPHS> StopBitsToRegConfig = { Configuration::Stop::BIT_1,
                                                                        Configuration::Stop::BIT_1_5,
                                                                        Configuration::Stop::BIT_2 };

  void initializeMapping()
  {
    usartObjects.fill( nullptr );
  }

  bool isUSART( const std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
        break;
      }
    }

    return result;
  }
}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */