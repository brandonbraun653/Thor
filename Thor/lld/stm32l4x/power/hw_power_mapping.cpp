/********************************************************************************
 *  File Name:
 *    hw_power_mapping.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/power/hw_power_mapping.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_PWR )

namespace Thor::LLD::PWR
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;

  /*-------------------------------------------------
  Module Functions
  -------------------------------------------------*/
  void initializeMapping()
  {
     PeripheralList[ POWER1_RESOURCE_INDEX ] = POWER1_PERIPH;
  }
}    // namespace Thor::LLD::POWER

#endif /* TARGET_STM32L4 && THOR_LLD_PWR */
