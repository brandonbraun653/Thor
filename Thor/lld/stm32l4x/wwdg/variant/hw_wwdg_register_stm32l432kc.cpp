/********************************************************************************
 *  File Name:
 *    hw_wwdg_register_stm32l432kc.cpp
 *
 *  Description:
 *    WWDG register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/wwdg>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/interface/wwdg/wwdg_intf.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/wwdg/hw_wwdg_types.hpp>
#include <Thor/lld/stm32l4x/wwdg/variant/hw_wwdg_register_stm32l4xxxx.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_WWDG )

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
                    Thor::LLD::WWDG::getResourceIndex };

  void WWDGInit()
  {
    using namespace Thor::LLD::WWDG;

/*------------------------------------------------
WWDG clock enable register access lookup table
------------------------------------------------*/
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    WWDG_ClockConfig[ WWDG1_RESOURCE_INDEX ].mask = //APB2ENR_WWDG1EN;
    WWDG_ClockConfig[ WWDG1_RESOURCE_INDEX ].reg  = //&RCC1_PERIPH->APB2ENR;
#endif

/*------------------------------------------------
WWDG reset register access lookup table
------------------------------------------------*/
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    WWDG_ResetConfig[ WWDG1_RESOURCE_INDEX ].mask = //APB2RSTR_WWDG1RST;
    WWDG_ResetConfig[ WWDG1_RESOURCE_INDEX ].reg  = //&RCC1_PERIPH->APB2RSTR;
#endif

/*------------------------------------------------
WWDG clocking bus source identifier
------------------------------------------------*/
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    WWDG_SourceClock[ WWDG1_RESOURCE_INDEX ] = //Chimera::Clock::Bus::APB2;
#endif
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_WWDG */
