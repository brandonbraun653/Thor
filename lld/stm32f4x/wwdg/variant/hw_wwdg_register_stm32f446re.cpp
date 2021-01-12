/********************************************************************************
 *  File Name:
 *    hw_wwdg_register_stm32f446re.cpp
 *
 *  Description:
 *    Explicit STM32F446xx WWDG data and routines
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/clock>
#include <Chimera/watchdog>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/rcc/rcc_types.hpp>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/stm32f4x/wwdg/variant/hw_wwdg_register_stm32f4xxxx.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_WWDG )

namespace Thor::LLD::WWDG
{

  void WWDGInit()
  {

  };

}    // namespace Thor::LLD::WWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WWDG */