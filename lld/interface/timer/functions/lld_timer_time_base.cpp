/******************************************************************************
 *  File Name:
 *    lld_timer_time_base.cpp
 *
 *  Description:
 *    Driver for time-base like operations
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cmath>
#include <Chimera/peripheral>
#include <Thor/lld/interface/inc/timer>
#include <Thor/lld/interface/inc/rcc>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Methods
  ---------------------------------------------------------------------------*/
  Chimera::Status_t setBaseTick( Handle_rPtr timer, const TickConfig &cfg )
  {
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    Turn off the timer in prep for modifications
    -------------------------------------------------------------------------*/
    disableCounter( timer );

    /*-------------------------------------------------------------------------
    Select the input clock source
    -------------------------------------------------------------------------*/
    switch ( cfg.clock_src )
    {
      case ClockSource::INTERNAL:
        SMS::clear( timer->mReg, SMCR_SMS );
        break;

      case ClockSource::EXTERNAL_MODE_1:
      case ClockSource::EXTERNAL_MODE_2:
      case ClockSource::INTERNAL_TRIGGER:
      default:
        RT_HARD_ASSERT( false );
        break;
    };

    /*-------------------------------------------------------------------------
    Calculate configuration values
    -------------------------------------------------------------------------*/
    uint32_t clockPrescale = 0;

    /* Get the timer core clock frequency */
    auto  rcc = RCC::getCoreClockCtrl();
    float finput =
        static_cast<float>( rcc->getPeriphClock( Type::PERIPH_TIMER, reinterpret_cast<std::uintptr_t>( timer->mReg ) ) );

    /* Translate the desired period into a frequency (Hz) */
    float fdesired = ( 1.0f / ( cfg.rate_ns * 1e9f ) );

    /* Calculate the closest divisor for TIMx prescaler, then check the error */
    float fdiv    = roundf( ( finput / fdesired ) + 1.0f );
    float factual = finput / ( fdiv + 1.0f );
    float error   = fabs( ( factual - fdesired ) / fdesired ) * 100.0f;

    if ( error > cfg.tolerance )
    {
      return Chimera::Status::FAIL;
    }
    else
    {
      clockPrescale = static_cast<uint32_t>( fdiv );
    }

    /*-------------------------------------------------------------------------
    Apply the calculated settings
    -------------------------------------------------------------------------*/
    if ( timer->mType == HardwareType::TIMER_HW_ADVANCED )
    {
      DIR::set( timer->mReg, CR1_DIR );
    }

    ARPE::set( timer->mReg, CR1_ARPE );           /* Buffer the auto reload register updates */
    AUTO_RELOAD::set( timer->mReg, reloadValue ); /* Set the reload value */
    PRESCALE::set( timer->mReg, clockPrescale );  /* Adjust the prescaler */
    COUNT::set( timer->mReg, reloadValue );       /* Reset counter to indicate a reset */

    return Chimera::Status::OK;
  }


  float getBaseTick( const Handle_rPtr timer )
  {

  }


  Chimera::Status_t setEventRate( Handle_rPtr timer, const float rate_ns )
  {
    // check sms register
  }


  float getEventRate( Handle_rPtr timer )
  {
    // check sms register
  }

}  // namespace Thor::LLD::TIMER
