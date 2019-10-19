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
    LookupTables::DMAInit();
    LookupTables::GPIOInit();
    LookupTables::SPIInit();
    LookupTables::UARTInit();
    LookupTables::USARTInit();
    LookupTables::WWDGInit();
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
