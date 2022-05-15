/******************************************************************************
 *  File Name:
 *    lld_timer_helpers.cpp
 *
 *  Description:
 *    Helpers for interacting with timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/inc/timer>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Methods
  ---------------------------------------------------------------------------*/
  Chimera::Status_t allocate( const Chimera::Timer::Instance &instance )
  {
    /*-------------------------------------------------------------------------
    Create the timer or grab the already allocated handle
    -------------------------------------------------------------------------*/
    Handle_rPtr handle = getHandle( instance );
    if( !handle )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Initialize the handle
    -------------------------------------------------------------------------*/
    handle->instance = instance;

    attach( handle, getPeriphRegister( instance ) );
    open( handle );

    return Chimera::Status::OK;
  }


  namespace Master
  {
    Chimera::Status_t initCore( Handle_rPtr timer, const Chimera::Timer::CoreConfig &cfg )
    {
      /*-------------------------------------------------------------------------
      Re-enable the timer
      -------------------------------------------------------------------------*/
      clockEnable( timer );
      reset( timer );

      /*-------------------------------------------------------------------------
      Set the base tick rate
      -------------------------------------------------------------------------*/
      TickConfig tick_cfg;
      tick_cfg.clock_src = ClockSource::INTERNAL;
      tick_cfg.rate_ns   = ( 1.0f / cfg.baseFreq ) * 1e9;

      return setBaseTickPeriod( timer, tick_cfg );
    }
  }

}  // namespace Thor::LLD::TIMER
