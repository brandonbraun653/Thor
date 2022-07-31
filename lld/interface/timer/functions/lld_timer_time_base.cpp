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
  Chimera::Status_t setBaseTickPeriod( Handle_rPtr timer, const TickConfig &cfg )
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
        SMS::clear( timer->registers, SMCR_SMS );
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
        static_cast<float>( rcc->getPeriphClock( Type::PERIPH_TIMER, reinterpret_cast<std::uintptr_t>( timer->registers ) ) );

    /* Translate the desired period into a frequency (Hz) */
    float fdesired = ( 1.0f / ( cfg.rate_ns / 1e9f ) );

    /* Calculate the closest divisor for TIMx prescaler, then check the error */
    float fdiv    = roundf( ( finput / fdesired ) ) - 1.0f;
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
    PRESCALE::set( timer->registers, clockPrescale << PSC_PSC_Pos );

    return Chimera::Status::OK;
  }


  float getBaseTickPeriod( const Handle_rPtr timer )
  {
    /*-------------------------------------------------------------------------
    Ensure the timer is being driven by an internal clock source
    -------------------------------------------------------------------------*/
    if ( SMS::get( timer->registers ) != 0 )
    {
      return RATE_UNKNOWN;
    }

    /*-------------------------------------------------------------------------
    Get the peripheral clock input period in nanoseconds
    -------------------------------------------------------------------------*/
    auto        rcc    = RCC::getCoreClockCtrl();
    const float finput = static_cast<float>(
        rcc->getPeriphClock( Chimera::Peripheral::Type::PERIPH_TIMER, reinterpret_cast<std::uintptr_t>( timer->registers ) ) );

    const float tinput_ns = ( 1.0 / finput ) * 1e9f;

    /*-------------------------------------------------------------------------
    Get the input clock prescaler
    -------------------------------------------------------------------------*/
    const float prescale = static_cast<float>( PRESCALE::get( timer->registers ) >> PSC_PSC_Pos );

    /*-------------------------------------------------------------------------
    Peripheral clock is scaled by TIMx_PSC + 1 (RM0394: 27.4.11)
    -------------------------------------------------------------------------*/
    return tinput_ns * ( prescale + 1.0f );
  }


  Chimera::Status_t setEventRate( Handle_rPtr timer, const float rate_ns )
  {
    /*-------------------------------------------------------------------------
    Ensure the timer is being driven by an internal clock source
    -------------------------------------------------------------------------*/
    if ( SMS::get( timer->registers ) != 0 )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Check the desired rate can be achieved
    -------------------------------------------------------------------------*/
    const float res_ns = getBaseTickPeriod( timer );
    if ( rate_ns < res_ns )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Calculate the closest reload value to achieve the given rate.
    -------------------------------------------------------------------------*/
    const uint32_t reloadValue = static_cast<uint32_t>( roundf( rate_ns / res_ns ) );

    if ( reloadValue > getMaxReload( timer->instance ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Apply the reload configuration
    -------------------------------------------------------------------------*/
    ARPE::set( timer->registers, CR1_ARPE );                          /* Buffer the auto reload register updates */
    AUTO_RELOAD::set( timer->registers, reloadValue << ARR_ARR_Pos ); /* Set the reload value */
    COUNT::set( timer->registers, 0 );                                /* Reset counter to indicate a reset */
    UG::set( timer->registers, EGR_UG );                              /* Generate update event to refresh hardware registers */

    return Chimera::Status::OK;
  }


  float getEventRate( Handle_rPtr timer )
  {
    /*-------------------------------------------------------------------------
    Ensure the timer is being driven by an internal clock source
    -------------------------------------------------------------------------*/
    if ( SMS::get( timer->registers ) != 0 )
    {
      return RATE_UNKNOWN;
    }

    /*-------------------------------------------------------------------------
    Pull the current configuration
    -------------------------------------------------------------------------*/
    const float reload = static_cast<float>( AUTO_RELOAD::get( timer->registers ) >> ARR_ARR_Pos );
    const float res_ns = getBaseTickPeriod( timer );

    return res_ns * reload;
  }


  uint32_t getAutoReload( Handle_rPtr timer )
  {
    return AUTO_RELOAD::get( timer->registers );
  }

}    // namespace Thor::LLD::TIMER
