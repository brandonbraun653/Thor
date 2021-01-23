/********************************************************************************
 *  File Name:
 *    hw_iwdg_driver_stm32f4.cpp
 *
 *  Description:
 *    Independent watchdog driver for the STM32F4 series family
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cmath>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/watchdog>
#include <Thor/lld/interface/inc/rcc>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_IWDG )

namespace Thor::LLD::Watchdog
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static IndependentDriver s_iwdg_drivers[ IWDG::NUM_IWDG_PERIPHS ];


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initializeIWDG()
  {
    /*-------------------------------------------------
    Attach all the expected mPeripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_iwdg_drivers, ARRAY_COUNT( s_iwdg_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  IndependentDriver_rPtr getDriver( const Chimera::Watchdog::IChannel channel )
  {
    if ( auto idx = getResourceIndex( channel ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_iwdg_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  IndependentDriver::IndependentDriver()
  {
  }


  IndependentDriver::~IndependentDriver()
  {
  }


  Chimera::Status_t IndependentDriver::attach( IRegisterMap *const mPeripheral )
  {
    /*------------------------------------------------
    Get mPeripheral descriptor settings
    ------------------------------------------------*/
    mPeriph        = mPeripheral;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( mPeripheral ) );

    return Chimera::Status::OK;
  }


  void IndependentDriver::enableClock()
  {
    auto rcc = Thor::LLD::RCC::getCoreClockCtrl();
    rcc->enableClock( Chimera::Clock::Bus::LSI );

    /*-------------------------------------------------
    The watchdog timer has to be started before any
    register access works.
    -------------------------------------------------*/
    this->start();
  }


  Chimera::Status_t IndependentDriver::setPrescaler( const Reg32_t val )
  {
    /*------------------------------------------------
    Wait for SR to indicate no ongoing HW updates.
    This can take at most 5 LSI clocks.
    ------------------------------------------------*/
    while ( IWDG::PVU::get( mPeriph ) )
    {
      continue;
    }

    /*------------------------------------------------
    Assign the register value with unlock-assign-lock
    ------------------------------------------------*/
    IWDG::KEY::set( mPeriph, IWDG::KR_UNLOCK );
    IWDG::PR::set( mPeriph, val );
    IWDG::KEY::set( mPeriph, IWDG::KR_LOCK );

    /*------------------------------------------------
    Wait again...
    ------------------------------------------------*/
    while ( IWDG::PVU::get( mPeriph ) )
    {
      continue;
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t IndependentDriver::setReload( const Reg32_t val )
  {
    /*------------------------------------------------
    Wait for SR to indicate no ongoing HW updates.
    This can take at most 125uS (5 clocks @ 40kHz LSI).
    ------------------------------------------------*/
    while ( IWDG::RVU::get( mPeriph ) )
    {
      continue;
    }

    /*------------------------------------------------
    Assign the register value with unlock-assign-lock
    ------------------------------------------------*/
    IWDG::KEY::set( mPeriph, IWDG::KR_UNLOCK );
    IWDG::RL::set( mPeriph, val );
    IWDG::KEY::set( mPeriph, IWDG::KR_LOCK );

    /*------------------------------------------------
    Wait again...
    ------------------------------------------------*/
    while ( IWDG::RVU::get( mPeriph ) )
    {
      continue;
    }

    return Chimera::Status::OK;
  }


  void IndependentDriver::start()
  {
    IWDG::KEY::set( mPeriph, IWDG::KR_START );
  }


  void IndependentDriver::reload()
  {
    IWDG::KEY::set( mPeriph, IWDG::KR_REFRESH );
  }


  size_t IndependentDriver::getMaxTimeout( const Reg32_t prescaler )
  {
    size_t base_period_us = static_cast<size_t>( ( 1000.0f * 1000.0f ) / static_cast<float>( IWDG::PERIPH_CLOCK_FREQ_HZ ) );
    size_t actual_period_us = base_period_us * prescaler;
    size_t timeout = actual_period_us * ( IWDG::COUNTER_MAX - IWDG::COUNTER_MIN );

    return timeout;
  }


  size_t IndependentDriver::getMinTimeout( const Reg32_t prescaler )
  {
    size_t base_period_us = static_cast<size_t>( ( 1000.0f * 1000.0f ) / static_cast<float>( IWDG::PERIPH_CLOCK_FREQ_HZ ) );
    size_t actual_period_us = base_period_us * prescaler;
    size_t timeout = actual_period_us * ( IWDG::COUNTER_MIN + 1 );

    return timeout;
  }


  size_t IndependentDriver::getTimeout()
  {
    size_t prescaler = IWDG::PR::get( mPeriph ) >> IWDG::PR_PR_Pos;
    size_t reloadVal = IWDG::RL::get( mPeriph ) >> IWDG::RLR_RL_Pos;

    size_t base_period_us = static_cast<size_t>( ( 1000.0f * 1000.0f ) / static_cast<float>( IWDG::PERIPH_CLOCK_FREQ_HZ ) );
    size_t actual_period_us = base_period_us * prescaler;
    size_t timeout = actual_period_us * ( reloadVal - IWDG::COUNTER_MIN );

    return timeout;
  }
}    // namespace Thor::LLD::IWDG

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
