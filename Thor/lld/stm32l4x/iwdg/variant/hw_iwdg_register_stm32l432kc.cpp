/********************************************************************************
 *  File Name:
 *    hw_iwdg_register_stm32l432kc.cpp
 *
 *  Description:
 *    IWDG register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/watchdog>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/iwdg/hw_iwdg_types.hpp>
#include <Thor/lld/stm32l4x/iwdg/variant/hw_iwdg_register_stm32l4xxxx.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_IWDG )

namespace Thor::LLD::IWDG
{
}    // namespace Thor::LLD::IWDG

namespace Thor::LLD::RCC::LookupTables
{
  /*-------------------------------------------------
  These tables are just place holders because the
  IWDG is clocked via the LSI.
  -------------------------------------------------*/


  /*------------------------------------------------
  Lookup tables for register access on a peripheral
  by peripheral basis.
  ------------------------------------------------*/
  RegisterConfig IWDG_ClockConfig[ Thor::LLD::IWDG::NUM_IWDG_PERIPHS ];
  RegisterConfig IWDG_ResetConfig[ Thor::LLD::IWDG::NUM_IWDG_PERIPHS ];
  Chimera::Clock::Bus IWDG_SourceClock[ Thor::LLD::IWDG::NUM_IWDG_PERIPHS ];

  PCC IWDGLookup = { IWDG_ClockConfig,
                    nullptr,
                    IWDG_ResetConfig,
                    IWDG_SourceClock,
                    Thor::LLD::IWDG::NUM_IWDG_PERIPHS,
                    Thor::LLD::Watchdog::getResourceIndex };

  void IWDGInit()
  {
    using namespace Thor::LLD::IWDG;

/*------------------------------------------------
IWDG clock enable register access lookup table
------------------------------------------------*/
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
    IWDG_ClockConfig[ IWDG1_RESOURCE_INDEX ].mask = 0;
    IWDG_ClockConfig[ IWDG1_RESOURCE_INDEX ].reg  = nullptr;
#endif

/*------------------------------------------------
IWDG reset register access lookup table
------------------------------------------------*/
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
    IWDG_ResetConfig[ IWDG1_RESOURCE_INDEX ].mask = 0;
    IWDG_ResetConfig[ IWDG1_RESOURCE_INDEX ].reg  = nullptr;
#endif

/*------------------------------------------------
IWDG clocking bus source identifier
------------------------------------------------*/
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
    IWDG_SourceClock[ IWDG1_RESOURCE_INDEX ] = Chimera::Clock::Bus::UNKNOWN_BUS;
#endif
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_IWDG */
