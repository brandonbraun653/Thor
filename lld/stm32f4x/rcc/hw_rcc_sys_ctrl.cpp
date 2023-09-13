/******************************************************************************
 *  File Name:
 *    hw_rcc_sys_ctrl.cpp
 *
 *  Description:
 *    System clock controller implementation
 *
 *  2021-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/assert>
#include <Chimera/clock>
#include <Chimera/common>
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/inc/flash>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/rcc>
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

    const size_t base_freq = getClockFrequency( registry->clockSource[ idx ] );
    if ( periph != Chimera::Peripheral::Type::PERIPH_TIMER )
    {
      return base_freq;
    }

    /*-----------------------------------------------------------------------
    Timer clocks have a special prescaler configuration that is dependent
    upon the bus they are attached to and the TIMPRE bit in the RCC_CFGR.
    -----------------------------------------------------------------------*/
    uint32_t prescaler = 0;
    if ( *registry->clockSource == Chimera::Clock::Bus::APB1 )
    {
      prescaler = PPRE1::get( RCC1_PERIPH );
    }
    else if ( *registry->clockSource == Chimera::Clock::Bus::APB2 )
    {
      prescaler = PPRE2::get( RCC1_PERIPH );
    }
    else
    {
      RT_HARD_ASSERT( false );
    }

    if ( TIMPRE::get( RCC1_PERIPH ) )
    {
      if ( prescaler <= CFGR_PPRE1_DIV4 )
      {
        return getClockFrequency( Chimera::Clock::Bus::HCLK );
      }
      else
      {
        return base_freq * 4;
      }
    }
    else
    {
      if ( prescaler >= CFGR_PPRE1_DIV2 )
      {
        return base_freq * 2;
      }
      else
      {
        return base_freq;
      }
    }
  }

}    // namespace Thor::LLD::RCC
