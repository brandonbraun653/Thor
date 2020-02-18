/********************************************************************************
 *   File Name:
 *    hw_wwdg_mapping.cpp
 *
 *   Description:
 *    Mappings for the watchdog timer resources
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/wwdg/hw_wwdg_mapping.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )
namespace Thor::Driver::WWDG
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;

  void initializeMapping()
  {
  
  
  }
  
  bool isWWDG( const std::uintptr_t address )
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

}
#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
