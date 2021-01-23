/********************************************************************************
 *  File Name:
 *    hw_rcc_sys_ctrl.cpp
 *
 *  Description:
 *    System clock controller implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/common/mapping/peripheral_mapping.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/flash>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static SystemClock s_system_clock;

  /*-------------------------------------------------------------------------------
  SystemClock Class Implementation
  -------------------------------------------------------------------------------*/
  SystemClock *getCoreClockCtrl()
  {
    return &s_system_clock;
  }


  SystemClock::SystemClock()
  {
    initialize();
  }


  SystemClock::~SystemClock()
  {
  }


  Chimera::Status_t SystemClock::configureProjectClocks()
  {
    /*------------------------------------------------
    Turn on the main internal regulator output voltage
    and set voltage scaling to achieve max clock.
    ------------------------------------------------*/
    PWR::setVoltageScaling( PWR::VoltageScale::SCALE_1 );

    /*-------------------------------------------------
    Set flash latency to a safe value for all possible
    clocks. This will slow down the configuration, but
    this is only performed once at startup.
    -------------------------------------------------*/
    FLASH::setLatency( 15 );

    /*------------------------------------------------
    Configure the system clocks to max performance
    ------------------------------------------------*/
    constexpr size_t hsiClkIn     = 16000000;     // 24 MHz
    constexpr size_t targetSysClk = 80000000;     // 80 MHz
    constexpr size_t targetVcoClk = 240000000;    // 240 MHz

    Chimera::Status_t cfgResult = Chimera::Status::OK;

    ClockTreeInit clkCfg;
    clkCfg.clear();

    clkCfg.enabled.hsi          = true;    // Needed for transfer of clock source
    clkCfg.enabled.lsi          = true;    // Allows IWDG use
    clkCfg.enabled.pll_core_clk = true;    // Will drive sys off PLL

    clkCfg.mux.pll = Chimera::Clock::Bus::HSI16;
    clkCfg.mux.sys = Chimera::Clock::Bus::PLLP;

    clkCfg.prescaler.ahb  = 1;
    clkCfg.prescaler.apb1 = 2;
    clkCfg.prescaler.apb2 = 2;

    cfgResult |= calculatePLLBaseOscillator( PLLType::CORE, hsiClkIn, targetVcoClk, clkCfg );
    cfgResult |= calculatePLLOuputOscillator( PLLType::CORE, PLLOut::P, targetVcoClk, targetSysClk, clkCfg );

    RT_HARD_ASSERT( cfgResult == Chimera::Status::OK );
    RT_HARD_ASSERT( configureClockTree( clkCfg ) );

    /*-------------------------------------------------
    Trim the flash latency back to a performant range
    now that the high speed clock has been configured.
    -------------------------------------------------*/
    FLASH::setLatency( 4 );

    return cfgResult;
  }


  Chimera::Status_t SystemClock::setCoreClockSource( const Chimera::Clock::Bus src )
  {
    /*-------------------------------------------------
    Only static clock configurations are used right now
    -------------------------------------------------*/
    return Chimera::Status::NOT_SUPPORTED;
  }

  size_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    auto registry = getPCCRegistry( periph );
    if ( !registry || !registry->clockSource || !registry->getResourceIndex )
    {
      return INVALID_CLOCK;
    }

    /*-------------------------------------------------
    Perform the lookup
    -------------------------------------------------*/
    size_t idx = registry->getResourceIndex( address );
    if ( idx == INVALID_RESOURCE_INDEX )
    {
      return INVALID_CLOCK;
    }

    return getClockFrequency( registry->clockSource[ idx ] );
  }

}    // namespace Thor::LLD::RCC
