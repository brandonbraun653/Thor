/********************************************************************************
 *  File Name:
 *    hw_flash_driver_stm32f4.cpp
 *
 *  Description:
 *    Implements the STM32F4 flash driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/flash/hw_flash_driver.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_FLASH )

namespace Thor::LLD::FLASH
{
  void initialize()
  {

  }
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_FLASH */