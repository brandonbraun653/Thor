/********************************************************************************
 *  File Name:
 *    hw_power_driver_STM32L4.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series POWER hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/power/hw_power_driver.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_mapping.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_prj.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_PWR )

namespace Thor::LLD::PWR
{
  /*-------------------------------------------------
  LLD->HLD Interface Implementation
  -------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();
    initializeMapping();

    return Chimera::Status::OK;
  }

}    // namespace Thor::LLD::POWER

#endif /* TARGET_STM32L4 && THOR_DRIVER_POWER */
