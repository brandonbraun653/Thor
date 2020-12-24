/********************************************************************************
 *  File Name:
 *    hw_wwdg_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the WWDG hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cmath>
#include <cstring>
#include <limits>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/clock>
#include <Chimera/common>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>
#include <Thor/lld/interface/watchdog/watchdog_prv_data.hpp>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/interface/watchdog/watchdog_types.hpp>
#include <Thor/lld/stm32l4x/wwdg/hw_wwdg_prj.hpp>
#include <Thor/lld/stm32l4x/wwdg/hw_wwdg_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_WWDG )

namespace Thor::LLD::Watchdog
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static WindowDriver s_wwdg_drivers[ NUM_WWDG_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  size_t calculate_pseudo_timeout( const Reg32_t prescaler, const Reg32_t counter )
  {
    static constexpr size_t MSK = 0x3F;    // 6-bit mask from equation

    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    bool validPrescaler = false;
    for ( auto x = 0; x < WWDG::NumPrescalers; x++ )
    {
      if ( prescaler == WWDG::DecimalPrescalers[ x ] )
      {
        validPrescaler = true;
        break;
      }
    }

    if ( !validPrescaler )
    {
      return 0;
    }

    /*-------------------------------------------------
    Get the current PCLK frequency
    -------------------------------------------------*/
    auto rcc        = Thor::LLD::RCC::getCoreClock();
    size_t pclk     = rcc->getClockFrequency( Chimera::Clock::Bus::PCLK1 );
    float clkPeriod = 1.0f / static_cast<float>( pclk );

    /*-------------------------------------------------
    Calculate the virtual countdown register value
    -------------------------------------------------*/
    float virtualT = static_cast<float>( ( counter & MSK ) + 1 );

    /*-------------------------------------------------
    Calculate psuedo timeout using RM 33.3.5
    -------------------------------------------------*/
    size_t timeout = static_cast<size_t>( clkPeriod * 4096.0f * pow( 2, prescaler ) * virtualT );
    return timeout;
  }


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initializeWWDG()
  {
    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_wwdg_drivers, ARRAY_COUNT( s_wwdg_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  WindowDriver_rPtr getDriver( const Chimera::Watchdog::WChannel channel )
  {
    if ( auto idx = getResourceIndex( channel ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_wwdg_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  size_t getWWDGClockFrequency()
  {
    /*-------------------------------------------------
    WWDG has a hardcoded divider built into the module
    -------------------------------------------------*/
    auto rcc    = Thor::LLD::RCC::getCoreClock();
    size_t pclk = rcc->getClockFrequency( Chimera::Clock::Bus::PCLK1 );
    return pclk / 4096;
  }


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  WindowDriver::WindowDriver() : mReload( WWDG::COUNTER_MAX )
  {
  }


  WindowDriver::~WindowDriver()
  {
  }


  Chimera::Status_t WindowDriver::attach( WRegisterMap *const peripheral )
  {
    /*------------------------------------------------
    Get peripheral descriptor settings
    ------------------------------------------------*/
    mPeriph        = peripheral;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    return Chimera::Status::OK;
  }


  void WindowDriver::enableClock()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_WWDG, mResourceIndex );
    rcc->reset( Chimera::Peripheral::Type::PERIPH_WWDG, mResourceIndex );
  }


  Chimera::Status_t WindowDriver::setPrescaler( const uint32_t val )
  {
    WWDG::WDGTB::set( mPeriph, val );
    return Chimera::Status::OK;
  }


  Chimera::Status_t WindowDriver::setReload( const uint32_t val )
  {
    /*-------------------------------------------------
    If OOR, set to a value that will almost immediately
    reset. Let's make some noise baby!
    -------------------------------------------------*/
    if ( ( val < WWDG::COUNTER_MIN ) || ( val > WWDG::COUNTER_MAX ) )
    {
      mReload = WWDG::COUNTER_MIN;
      return Chimera::Status::INVAL_FUNC_PARAM;
    }
    else
    {
      mReload = val;
      return Chimera::Status::OK;
    }
  }


  Chimera::Status_t WindowDriver::setWindow( const uint8_t percent )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if ( !percent || percent > 100 )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Set the window based on the current reload value
    -------------------------------------------------*/
    float pct     = 1.0f / static_cast<float>( percent );
    float rld_rng = static_cast<float>( mReload - WWDG::COUNTER_MIN );

    Reg32_t regVal = static_cast<Reg32_t>( rld_rng * pct ) + WWDG::COUNTER_MIN;
    WWDG::W::set( mPeriph, regVal );

    return Chimera::Status::OK;
  }


  void WindowDriver::start()
  {
    /*-------------------------------------------------
    Manually write the register to avoid generating a
    reset with the REG_ACCESSOR macro, which will set
    the T6 bit to zero.

    Only write if previously haven't. Don't want to
    accidentally reload the watchdog.
    -------------------------------------------------*/
    if ( !WWDG::WDGA::get( mPeriph ) )
    {
      mReload |= WWDG::CR_T_6;
      mPeriph->CR = ( WWDG::CR_WDGA ) | ( mReload & WWDG::CR_T_Msk );
    }
  }


  void WindowDriver::reload()
  {
    WWDG::T::set( mPeriph, mReload << WWDG::CR_T_Pos );
  }


  size_t WindowDriver::getMaxTimeout( const uint32_t prescaler )
  {
    return calculate_pseudo_timeout( prescaler, WWDG::COUNTER_MAX );
  }


  size_t WindowDriver::getMinTimeout( const uint32_t prescaler )
  {
    return calculate_pseudo_timeout( prescaler, WWDG::COUNTER_MIN );
  }


  size_t WindowDriver::getTimeout()
  {
    size_t prescaler = WWDG::WDGTB::get( mPeriph ) >> WWDG::CR_WDGA_Pos;
    return calculate_pseudo_timeout( prescaler, mReload );
  }
}    // namespace Thor::LLD::Watchdog

#endif /* TARGET_STM32L4 && THOR_DRIVER_WWDG */
