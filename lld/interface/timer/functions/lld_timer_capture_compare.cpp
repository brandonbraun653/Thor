/******************************************************************************
 *  File Name:
 *    lld_timer_capture_compare.cpp
 *
 *  Description:
 *    Capture compare aspects of a timer driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <Chimera/common>
#include <Thor/lld/interface/inc/timer>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t disableCCChannel( Handle_rPtr timer, const Chimera::Timer::Channel ch )
  {
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        CC1E::clear( timer->registers, CCER_CC1E );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        CC2E::clear( timer->registers, CCER_CC2E );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        CC3E::clear( timer->registers, CCER_CC3E );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        CC4E::clear( timer->registers, CCER_CC4E );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
    };

    return Chimera::Status::OK;
  }


  Chimera::Status_t enableCCChannel( Handle_rPtr timer, const Chimera::Timer::Channel ch )
  {
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        CC1E::set( timer->registers, CCER_CC1E );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        CC2E::set( timer->registers, CCER_CC2E );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        CC3E::set( timer->registers, CCER_CC3E );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        CC4E::set( timer->registers, CCER_CC4E );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
    };

    return Chimera::Status::OK;
  }


  Chimera::Status_t setCCPolarity( Handle_rPtr timer, const Chimera::Timer::Channel ch, const CCPolarity pol )
  {
    /*-------------------------------------------------------------------------
    Decide the bit mask to be applied to the mode
    -------------------------------------------------------------------------*/
    const CCMode mode    = getCCMode( timer, ch );
    uint32_t     reg_val = 0;

    if ( mode == CCM_OUTPUT )
    {
      reg_val = pol;
    }
    else
    {
      switch ( pol )
      {
        case CCPolarity::CCP_IN_RISING_EDGE:
          reg_val = 0;
          break;

        case CCPolarity::CCP_IN_FALLING_EDGE:
          reg_val = 1;
          break;

        case CCPolarity::CCP_IN_BOTH_EDGE:
          reg_val = 3;
          break;

        default:
          return Chimera::Status::NOT_SUPPORTED;
      };
    }

    /*-------------------------------------------------------------------------
    Apply the polarity configuration
    -------------------------------------------------------------------------*/
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        CC1P::set( timer->registers, ( reg_val & 0x1 ) << CCER_CC1P_Pos );
        if ( reg_val == 3 )
        {
          CC1NP::set( timer->registers, 1u << CCER_CC1NP_Pos );
        }
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        CC2P::set( timer->registers, ( reg_val & 0x1 ) << CCER_CC2P_Pos );
        if ( reg_val == 3 )
        {
          CC2NP::set( timer->registers, 1u << CCER_CC2NP_Pos );
        }
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        CC3P::set( timer->registers, ( reg_val & 0x1 ) << CCER_CC3P_Pos );
        if ( reg_val == 3 )
        {
          CC3NP::set( timer->registers, 1u << CCER_CC3NP_Pos );
        }
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        CC4P::set( timer->registers, ( reg_val & 0x1 ) << CCER_CC4P_Pos );
        if ( reg_val == 3 )
        {
          CC4NP::set( timer->registers, 1u << CCER_CC4NP_Pos );
        }
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
    };

    return Chimera::Status::OK;
  }


  Chimera::Status_t setCCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch, const CCMode mode )
  {
    /*-------------------------------------------------------------------------
    Set the mode depending on the input channel
    -------------------------------------------------------------------------*/
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        CC1S::set( timer->registers, mode << CCMR1_CC1S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        CC2S::set( timer->registers, mode << CCMR1_CC2S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        CC3S::set( timer->registers, mode << CCMR2_CC3S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        CC4S::set( timer->registers, mode << CCMR2_CC4S_Pos );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
    };

    return Chimera::Status::OK;
  }


  CCMode getCCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch )
  {
    switch ( ch )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        return static_cast<CCMode>( CC1S::get( timer->registers ) >> CCMR1_CC1S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        return static_cast<CCMode>( CC2S::get( timer->registers ) >> CCMR1_CC2S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        return static_cast<CCMode>( CC3S::get( timer->registers ) >> CCMR2_CC3S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        return static_cast<CCMode>( CC4S::get( timer->registers ) >> CCMR2_CC4S_Pos );
        break;

      default:
        return CCM_INVALID;
    };
  }

}    // namespace Thor::LLD::TIMER
