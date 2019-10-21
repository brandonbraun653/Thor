/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping.cpp
 *
 *   Description:
 *    Useful maps for the RCC peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_RCC == 1 )

namespace Thor::Driver::RCC
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;


  void initializeMapping()
  {

#if defined( THOR_DRIVER_DMA ) && ( THOR_DRIVER_DMA == 1 )
    LookupTables::DMAInit();
#endif

#if defined( THOR_DRIVER_GPIO ) && ( THOR_DRIVER_GPIO == 1 )
    LookupTables::GPIOInit();
#endif

#if defined( THOR_DRIVER_SPI ) && ( THOR_DRIVER_SPI == 1 )
    LookupTables::SPIInit();
#endif 

#if defined( THOR_DRIVER_UART ) && ( THOR_DRIVER_UART == 1 )
    LookupTables::UARTInit();
#endif

#if defined( THOR_DRIVER_USART ) && ( THOR_DRIVER_USART == 1 )
    LookupTables::USARTInit();
#endif

#if defined( THOR_DRIVER_WWDG ) && ( THOR_DRIVER_WWDG == 1 )
    LookupTables::WWDGInit();
#endif
  }

  bool isRCC( const std::uintptr_t address )
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

}    // namespace Thor::Driver::RCC

#endif /* TARGET_STM32F4 && THOR_DRIVER_RCC */
