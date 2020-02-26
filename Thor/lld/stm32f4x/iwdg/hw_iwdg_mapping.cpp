/********************************************************************************
 *  File Name:
 *    hw_iwdg_mapping.cpp
 *
 *  Description:
 *    Mappings for the watchdog timer resources
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_mapping.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_IWDG )
namespace Thor::LLD::IWDG
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
