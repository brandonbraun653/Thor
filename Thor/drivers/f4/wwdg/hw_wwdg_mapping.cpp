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


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WATCHDOG == 1 )
namespace Thor::Driver::WWDG
{
  const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex = {
    { reinterpret_cast<std::uintptr_t>( WWDG_PERIPH ), 0u } 
  };
}
#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
