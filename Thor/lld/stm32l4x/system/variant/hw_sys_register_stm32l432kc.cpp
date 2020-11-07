/********************************************************************************
 *  File Name:
 *    hw_sys_register_stm32l432kc.cpp
 *
 *  Description:
 *    SYSCFG register definitions for STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/system/sys_intf.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/system/variant/hw_sys_register_stm32l432kc.hpp>

namespace Thor::LLD::SYS
{

}  // namespace Thor::LLD::SYS


namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_gpio_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig SYSCFG_ClockConfig[ Thor::LLD::SYS::NUM_SYSCFG_PERIPHS ];
  RegisterConfig SYSCFG_ResetConfig[ Thor::LLD::SYS::NUM_SYSCFG_PERIPHS ];
  Chimera::Clock::Bus SYSCFG_SourceClock[ Thor::LLD::SYS::NUM_SYSCFG_PERIPHS ];

  PCC SYSCFGLookup = { SYSCFG_ClockConfig,
                     nullptr,
                     SYSCFG_ResetConfig,
                     SYSCFG_SourceClock,
                     Thor::LLD::SYS::NUM_SYSCFG_PERIPHS,
                     Thor::LLD::SYS::getResourceIndex };

  void SYSCFGInit()
  {
    using namespace Thor::LLD::SYS;

    /*------------------------------------------------
    SYSCFG clock enable register access lookup table
    ------------------------------------------------*/
    SYSCFG_ClockConfig[ SYSCFG1_RESOURCE_INDEX ].mask = APB2ENR_SYSCFGEN;
    SYSCFG_ClockConfig[ SYSCFG1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;

    /*------------------------------------------------
    SYSCFG reset register access lookup table
    ------------------------------------------------*/
    SYSCFG_ResetConfig[ SYSCFG1_RESOURCE_INDEX ].mask = APB2RSTR_SYSCFGRST;
    SYSCFG_ResetConfig[ SYSCFG1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;

    /*------------------------------------------------
    SYSCFG clocking bus source identifier
    ------------------------------------------------*/
    SYSCFG_SourceClock[ SYSCFG1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB2;
  };

}    // namespace Thor::LLD::RCC::LookupTables
