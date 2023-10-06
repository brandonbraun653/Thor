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
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Special computations to get the timer peripheral clock.
   *
   * Timer clocks have a special prescaler configuration that is dependent
   * upon the bus they are attached to and the TIMPRE bit in the RCC_CFGR.
   *
   * @param registry    The peripheral registry to use
   * @param base_clock  The base clock frequency of the bus the timer is attached to
   * @return size_t     The actual timer peripheral clock frequency
   */
  static size_t get_timer_periph_clock( const Thor::LLD::RCC::PCC *registry, const size_t base_clock )
  {
    /*-------------------------------------------------------------------------
    Get the prescaler for the bus the timer is attached to
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    Compute the actual timer clock frequency
    -------------------------------------------------------------------------*/
    if ( TIMPRE::get( RCC1_PERIPH ) )
    {
      if ( prescaler <= CFGR_PPRE1_DIV4 )
      {
        return s_system_clock.getClockFrequency( Chimera::Clock::Bus::HCLK );
      }
      else
      {
        return base_clock * 4;
      }
    }
    else
    {
      if ( prescaler >= CFGR_PPRE1_DIV2 )
      {
        return base_clock * 2;
      }
      else
      {
        return base_clock;
      }
    }
  }


  /**
   * @brief Gets the frequency of the SDIO peripheral clock.
   *
   * In the F4 series, the SDIO peripheral clock can be sourced from either the
   * system clock or the PLL48CLK, which is in turn driven by either the PLLQ or
   * PLLSAI_P clocks. This function will determine which clock is actually driving
   * the SDIO peripheral and return its current frequency.
   *
   * @return size_t
   */
  static size_t get_sdio_periph_clock()
  {
    /*-------------------------------------------------------------------------
    Connected to the system clock?
    -------------------------------------------------------------------------*/
    if( SDIOSEL::get( RCC1_PERIPH ) & DCKCFGR2_SDIOSEL )
    {
      return s_system_clock.getClockFrequency( Chimera::Clock::Bus::SYSCLK );
    }

    /*-------------------------------------------------------------------------
    Connected to the PLL48CLK. Double check the configuration.
    -------------------------------------------------------------------------*/
    size_t actual_clock = 0;

    if( CK48MSEL::get( RCC1_PERIPH ) & DCKCFGR2_CK48MSEL )
    {
      actual_clock = s_system_clock.getClockFrequency( Chimera::Clock::Bus::PLLSAI_P );
    }
    else
    {
      actual_clock = s_system_clock.getClockFrequency( Chimera::Clock::Bus::PLLQ );
    }

    RT_HARD_ASSERT( actual_clock == 48000000 );
    return actual_clock;
  }

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

    switch( periph )
    {
      case Chimera::Peripheral::Type::PERIPH_TIMER:
        return get_timer_periph_clock( registry, getClockFrequency( registry->clockSource[ idx ] ) );

      case Chimera::Peripheral::Type::PERIPH_SDIO:
        return get_sdio_periph_clock();

      default:
        return getClockFrequency( registry->clockSource[ idx ] );
    }
  }

}    // namespace Thor::LLD::RCC
