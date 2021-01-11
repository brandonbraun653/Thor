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
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/stm32f4x/wwdg/variant/hw_wwdg_register_stm32f4xxxx.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_mapping.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_WWDG )

namespace Thor::LLD::WWDG
{
}    // namespace Thor::LLD::WWDG

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral
  by peripheral basis.
  ------------------------------------------------*/
  RegisterConfig WWDG_ClockConfig[ Thor::LLD::WWDG::NUM_WWDG_PERIPHS ];
  RegisterConfig WWDG_ResetConfig[ Thor::LLD::WWDG::NUM_WWDG_PERIPHS ];
  Chimera::Clock::Bus WWDG_SourceClock[ Thor::LLD::WWDG::NUM_WWDG_PERIPHS ];

  PCC WWDGLookup = { WWDG_ClockConfig,
                     nullptr,
                     WWDG_ResetConfig,
                     WWDG_SourceClock,
                     Thor::LLD::WWDG::NUM_WWDG_PERIPHS,
                     Thor::LLD::Watchdog::getResourceIndex };

  void WWDGInit()
  {
    using namespace Thor::LLD::WWDG;

/*------------------------------------------------
WWDG clock enable register access lookup table
------------------------------------------------*/
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    WWDG_ClockConfig[ WWDG1_RESOURCE_INDEX ].mask = APB1ENR_WWDGEN;
    WWDG_ClockConfig[ WWDG1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR;
#endif

/*------------------------------------------------
WWDG reset register access lookup table
------------------------------------------------*/
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    WWDG_ResetConfig[ WWDG1_RESOURCE_INDEX ].mask = 0;
    WWDG_ResetConfig[ WWDG1_RESOURCE_INDEX ].reg  = nullptr;
#endif

/*------------------------------------------------
WWDG clocking bus source identifier
------------------------------------------------*/
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    WWDG_SourceClock[ WWDG1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
#endif
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* TARGET_STM32F4 && THOR_DRIVER_WWDG */