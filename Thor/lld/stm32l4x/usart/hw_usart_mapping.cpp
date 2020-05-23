/********************************************************************************
 *  File Name:
 *    hw_usart_mapping.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/usart/hw_usart_mapping.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;
  DriverInstanceList usartObjects;

  /*-------------------------------------------------
  Module Functions 
  -------------------------------------------------*/
  void initializeMapping()
  {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
    PeripheralList[ USART1_RESOURCE_INDEX ] = USART1_PERIPH;
#endif

#if defined( STM32_USART2_PERIPH_AVAILABLE )
    PeripheralList[ USART2_RESOURCE_INDEX ] = USART2_PERIPH;
#endif 

#if defined( STM32_USART3_PERIPH_AVAILABLE )
    PeripheralList[ USART3_RESOURCE_INDEX ] = USART3_PERIPH;
#endif

    usartObjects.fill( nullptr );
  }

  /*-------------------------------------------------
  Initialize the Chimera Option to Register Config Mappings
  -------------------------------------------------*/
  /* clang-format off */
  std::array<uint32_t, static_cast<size_t>( Chimera::Serial::CharWid::NUM_OPTIONS )> CharWidToRegConfig = { 
    Configuration::WordLength::LEN_8BIT,
    Configuration::WordLength::LEN_9BIT 
  };

  std::array<uint32_t, static_cast<size_t>( Chimera::Serial::Parity::NUM_OPTIONS )> ParityToRegConfig = { 
    Configuration::Parity::NONE, 
    Configuration::Parity::EVEN,
    Configuration::Parity::ODD 
  };

  std::array<uint32_t, static_cast<size_t>( Chimera::Serial::StopBits::NUM_OPTIONS )> StopBitsToRegConfig = { 
    Configuration::Stop::BIT_1,
    Configuration::Stop::BIT_1_5,
    Configuration::Stop::BIT_2 
  };
  /* clang-format on */
}    // namespace Thor::LLD::USART

#endif /* TARGET_STM32L4 && THOR_LLD_USART */
