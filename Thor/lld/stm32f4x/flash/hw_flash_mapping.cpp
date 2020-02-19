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
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/flash/hw_flash_mapping.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_FLASH == 1 )

namespace Thor::Driver::Flash
{
#if defined( EMBEDDED )
  RegisterMap *const FLASH_PERIPH = reinterpret_cast<RegisterMap *const>( FLASH_BASE_ADDR );

#elif defined( _SIM )
  RegisterMap *const FLASH_PERIPH = new RegisterMap;

#endif

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_FLASH */