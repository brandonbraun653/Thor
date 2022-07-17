/******************************************************************************
 *  File Name:
 *    lld_timer_break.cpp
 *
 *  Description:
 *    Break and dead time register control
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
  Public Functions
  ---------------------------------------------------------------------------*/
  void generateBreakEvent( Handle_rPtr timer, const BreakChannel channel )
  {
    switch( channel )
    {
      case BreakChannel::BREAK_INPUT_1:
        BG::set( timer->registers, EGR_BG );
        break;

      case BreakChannel::BREAK_INPUT_2:
        B2G::set( timer->registers, EGR_B2G );
        break;

      default:
        // Do nothing
        break;
    };
  }


  void breakEnable( Handle_rPtr timer, const BreakSource src, const BreakChannel channel )
  {
    /*-------------------------------------------------------------------------
    As far as I'm aware, only TIM1 has this functionality in Thor STM32 chips
    -------------------------------------------------------------------------*/
#if defined( DEBUG )
    RT_HARD_ASSERT( timer->instance == Chimera::Timer::Instance::TIMER1 );
#endif

    if ( src == BREAK_SOURCE_INTERNAL )
    {
      switch ( channel )
      {
        case BreakChannel::BREAK_INPUT_1:
          BKE::set( timer->registers, BDTR_BKE );
          break;

        case BreakChannel::BREAK_INPUT_2:
          BK2E::set( timer->registers, BDTR_BK2E );
          break;

        default:
          // Do nothing
          break;
      };
    }
    else    // BREAK_SOURCE_EXTERNAL
    {
      RT_HARD_ASSERT( false );
    }
  }


  void setBreakPolarity( Handle_rPtr timer, const BreakSource src, const BreakChannel channel, const BreakPolarity polarity )
  {
    /*-------------------------------------------------------------------------
    As far as I'm aware, only TIM1 has this functionality in Thor STM32 chips
    -------------------------------------------------------------------------*/
#if defined( DEBUG )
    RT_HARD_ASSERT( timer->instance == Chimera::Timer::Instance::TIMER1 );
#endif

    if ( src == BREAK_SOURCE_INTERNAL )
    {
      switch ( channel )
      {
        case BreakChannel::BREAK_INPUT_1:
          BKP::set( timer->registers, ( polarity << BDTR_BKP_Pos ) );
          break;

        case BreakChannel::BREAK_INPUT_2:
          BK2P::set( timer->registers, ( polarity << BDTR_BK2P_Pos ) );
          break;

        default:
          // Do nothing
          break;
      };
    }
    else    // BREAK_SOURCE_EXTERNAL
    {
      RT_HARD_ASSERT( false );
    }
  }


  void lockoutTimer( Handle_rPtr timer, const LockoutLevel level )
  {
    LOCK::set( timer->registers, ( level << BDTR_LOCK_Pos ) );
  }


  bool isLockedOut( Handle_rPtr timer )
  {
    return ( LOCK::get( timer->registers ) >> BDTR_LOCK_Pos ) != LOCK_LEVEL_OFF;
  }


  bool setDeadTime( Handle_rPtr timer, const float dt_ns )
  {
    // TODO BMB: Fill this out more intelligently. Need to just see if the timer works for now.
    DTG::set( timer->registers, 0 );    // minimal dead-time
    return true;
  }

}    // namespace Thor::LLD::TIMER
