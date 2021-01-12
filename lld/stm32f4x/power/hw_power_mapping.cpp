/********************************************************************************
 *  File Name:
 *    hw_power_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/power/hw_power_mapping.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_PWR )

namespace Thor::LLD::PWR
{
#if defined( EMBEDDED )
  RegisterMap *const PWR_PERIPH = reinterpret_cast<RegisterMap *const>( PWR_BASE_ADDR );
#endif

}    // namespace Thor::LLD::PWR

#endif /* TARGET_STM32F4 && THOR_DRIVER_PWR */
