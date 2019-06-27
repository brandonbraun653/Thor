/********************************************************************************
 *   File Name:
 *    hw_rcc_driver_stm32f4.cpp
 *
 *   Description:
 *    Implements the low level driver for the Reset and Clock Control peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/


/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>

namespace Thor::Driver::RCC
{
  
  
  
  Chimera::Status_t enablePeripheralClock( const Chimera::Peripheral::Type periph, const size_t instance )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    // TODO: Need to do better than a switch statement here. Maybe pointer to base class that has members
    // TODO: called clock config, periph reset etc?? I need to register these memory locations on init 
    // TODO: so that lookup is a breeze at runtime.

    return result;
  }
}    // namespace Thor::Driver::RCC
