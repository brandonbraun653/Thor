/******************************************************************************
 *  File Name:
 *    hw_rcc_sys_ctrl.cpp
 *
 *  Description:
 *    System clock controller implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/flash>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prv.hpp>

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static SystemClock s_system_clock;

  /*---------------------------------------------------------------------------
  SystemClock Class Implementation
  ---------------------------------------------------------------------------*/
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

  void SystemClock::enableClock( const Chimera::Clock::Bus clock )
  {
    auto isrMask = INT::disableInterrupts();

    switch ( clock )
    {
      case Chimera::Clock::Bus::HSE:
        HSEON::set( RCC1_PERIPH, CR_HSEON );
        while ( !HSERDY::get( RCC1_PERIPH ) )
        {
          ;
        }
        break;

      case Chimera::Clock::Bus::HSI16:
        enableHSI();
        break;

      case Chimera::Clock::Bus::LSI:
        enableLSI();
        break;

      default:
        RT_HARD_ASSERT( false );
        break;
    }

    INT::enableInterrupts( isrMask );
  }


  Chimera::Status_t SystemClock::configureProjectClocks()
  {
    /*-------------------------------------------------------------------------
    Set flash latency to a safe value for all possible
    clocks. This will slow down the configuration, but
    this is only performed once at startup.
    -------------------------------------------------------------------------*/
    FLASH::setLatency( 15 );

    /*-------------------------------------------------------------------------
    Not strictly necessary, but done because this
    config function uses the max system clock.
    -------------------------------------------------------------------------*/
    PWR::setOverdriveMode( true );

    /*-------------------------------------------------------------------------
    Configure the system clocks to max performance
    -------------------------------------------------------------------------*/
    constexpr size_t hsiClkIn     = 16000000;            // 16 MHz
    constexpr size_t targetSysClk = 120000000;           // 120 MHz
    constexpr size_t targetUSBClk = 48000000;            // 48 MHz
    constexpr size_t targetVcoClk = 2 * targetSysClk;    // 240 MHz

    Chimera::Status_t cfgResult = Chimera::Status::OK;

    ClockTreeInit clkCfg;
    clkCfg.clear();

    /* Select which clocks to turn on  */
    clkCfg.enabled.hsi          = true;    // Needed for transfer of clock source
    clkCfg.enabled.lsi          = true;    // Allows IWDG use
    clkCfg.enabled.pll_core_clk = true;    // Will drive sys off PLL
    clkCfg.enabled.pll_core_q   = true;    // USB 48 MHz clock

    /* Select clock mux routing */
    clkCfg.mux.pll   = Chimera::Clock::Bus::HSI16;
    clkCfg.mux.sys   = Chimera::Clock::Bus::PLLP;
    clkCfg.mux.usb48 = Chimera::Clock::Bus::PLLQ;

    /* Divisors from the system clock */
    clkCfg.prescaler.ahb  = 1;
    clkCfg.prescaler.apb1 = 4;
    clkCfg.prescaler.apb2 = 2;

    /* Figure out PLL configuration settings */
    cfgResult |= calculatePLLBaseOscillator( PLLType::CORE, hsiClkIn, targetVcoClk, clkCfg );
    cfgResult |= calculatePLLOuputOscillator( PLLType::CORE, PLLOut::P, targetVcoClk, targetSysClk, clkCfg );
    cfgResult |= calculatePLLOuputOscillator( PLLType::CORE, PLLOut::Q, targetVcoClk, targetUSBClk, clkCfg );

    RT_HARD_ASSERT( cfgResult == Chimera::Status::OK );
    RT_HARD_ASSERT( configureClockTree( clkCfg ) );

    /*-------------------------------------------------------------------------
    Verify the user's target clocks have been achieved
    -------------------------------------------------------------------------*/
    size_t sys_clk = getSystemClock();
    RT_HARD_ASSERT( sys_clk == targetSysClk );

    /*-------------------------------------------------------------------------
    Trim the flash latency back to a performant range
    now that the high speed clock has been configured.
    -------------------------------------------------------------------------*/
    FLASH::setLatency( FLASH::LATENCY_AUTO_DETECT );

    /*-------------------------------------------------------------------------
    Make sure the rest of the system knows about the
    new clock frequency.
    -------------------------------------------------------------------------*/
    CortexM4::Clock::updateCoreClockCache( sys_clk );

    return cfgResult;
  }


  Chimera::Status_t SystemClock::setCoreClockSource( const Chimera::Clock::Bus src )
  {
    return select_system_clock_source( src );
  }


  size_t SystemClock::getClockFrequency( const Chimera::Clock::Bus clock )
  {
    return getBusFrequency( clock );
  }


  size_t SystemClock::getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address )
  {
    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    auto registry = getPCCRegistry( periph );
    if ( !registry || !registry->clockSource || !registry->getResourceIndex )
    {
      return INVALID_CLOCK;
    }

    /*-------------------------------------------------------------------------
    Perform the lookup
    -------------------------------------------------------------------------*/
    size_t idx = registry->getResourceIndex( address );
    if ( idx == INVALID_RESOURCE_INDEX )
    {
      return INVALID_CLOCK;
    }

    return getClockFrequency( registry->clockSource[ idx ] );
  }

}    // namespace Thor::LLD::RCC
