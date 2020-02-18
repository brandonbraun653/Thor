/********************************************************************************
 *   File Name:
 *    hw_iwdg_mapping.cpp
 *
 *   Description:
 *    Mappings for the watchdog timer resources
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/iwdg/hw_iwdg_mapping.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_IWDG == 1 )
namespace Thor::Driver::IWDG
{
/*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;
  DriverInstanceList iwdgObjects;

  void initializeMapping()
  {
    iwdgObjects.fill( nullptr );
  }

  bool isIWDG( const std::uintptr_t address )
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
}
#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
