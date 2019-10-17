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
#if defined( _EMBEDDED )
  RegisterMap *const WWDG_PERIPH = reinterpret_cast<RegisterMap *const>( WWDG_BASE_ADDR );

#elif defined( _SIM )
  RegisterMap *const WWDG_PERIPH = new RegisterMap;

#endif


  const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex = {
    { reinterpret_cast<std::uintptr_t>( WWDG_PERIPH ), 0u } 
  };
}
#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
