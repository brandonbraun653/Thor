/********************************************************************************
 *   File Name:
 *    hw_flash_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/flash/hw_flash_mapping.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_FLASH )

namespace Thor::LLD::FLASH
{
#if defined( EMBEDDED )
  RegisterMap *const FLASH_PERIPH = reinterpret_cast<RegisterMap *const>( FLASH_BASE_ADDR );

#elif defined( _SIM )
  RegisterMap *const FLASH_PERIPH = new RegisterMap;

#endif

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_FLASH */