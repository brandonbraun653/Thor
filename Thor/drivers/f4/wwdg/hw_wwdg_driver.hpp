/********************************************************************************
 *   File Name:
 *    hw_wwdg_driver.hpp
 *
 *   Description:
 *    Declares the low level hardware watchdog driver interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_WWDG_DRIVER_HPP
#define THOR_HW_WWDG_DRIVER_HPP

/* Driver Includes */
#include <Thor/headers.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WATCHDOG == 1 )
namespace Thor::Driver::WWDG
{
  class Driver
  {
  public:

  private:

  };
}    // namespace Thor::Driver::WWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_WWDG_DRIVER_HPP */