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
  Static Data
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void setOutputIdleState( Handle_rPtr timer, const Chimera::Timer::Output ch, const Chimera::GPIO::State state )
  {
    if( state == Chimera::GPIO::State::HIGH )
    {
      CR2::set( timer->registers, IdleFlags[ EnumValue( ch ) ] );
    }
    else
    {
      CR2::clear( timer->registers, IdleFlags[ EnumValue( ch ) ] );
    }
  }


  void setOutputIdleStateBulk( Handle_rPtr timer, const uint32_t bf, const Chimera::GPIO::State state )
  {
    /*-------------------------------------------------------------------------
    Build up the bit mask
    -------------------------------------------------------------------------*/
    uint32_t bitMask = 0;
    for( size_t idx = 0; idx < IdleFlags.size(); idx++ )
    {
      if( bf & ( 1u << idx ) )
      {
        bitMask |= IdleFlags[ idx ];
      }
    }

    /*-------------------------------------------------------------------------
    Assign the new fields
    -------------------------------------------------------------------------*/
    if( state == Chimera::GPIO::State::HIGH )
    {
      CR2::set( timer->registers, bitMask );
    }
    else
    {
      CR2::clear( timer->registers, bitMask );
    }
  }


  void disableCCOutput( Handle_rPtr timer, const Chimera::Timer::Output ch )
  {
    RT_DBG_ASSERT( ch >= Chimera::Timer::Output::NUM_OPTIONS );
    CCER::clear( timer->registers, EnableFlags[ EnumValue( ch ) ] );
  }


  void disableCCOutputBulk( Handle_rPtr timer, const uint32_t bf )
  {
    /*-------------------------------------------------------------------------
    Build up the bit mask
    -------------------------------------------------------------------------*/
    uint32_t bitMask = 0;
    for( size_t idx = 0; idx < EnableFlags.size(); idx++ )
    {
      if( bf & ( 1u << idx ) )
      {
        bitMask |= EnableFlags[ idx ];
      }
    }

    /*-------------------------------------------------------------------------
    Enable the outputs
    -------------------------------------------------------------------------*/
    CCER::clear( timer->registers, bitMask );
  }


  void enableCCOutput( Handle_rPtr timer, const Chimera::Timer::Output ch )
  {
    RT_DBG_ASSERT( ch >= Chimera::Timer::Output::NUM_OPTIONS );
    CCER::setbit( timer->registers, EnableFlags[ EnumValue( ch ) ] );
  }


  void enableCCOutputBulk( Handle_rPtr timer, const uint32_t bf )
  {
    /*-------------------------------------------------------------------------
    Build up the bit mask
    -------------------------------------------------------------------------*/
    uint32_t bitMask = 0;
    for( size_t idx = 0; idx < EnableFlags.size(); idx++ )
    {
      if( bf & ( 1u << idx ) )
      {
        bitMask |= EnableFlags[ idx ];
      }
    }

    /*-------------------------------------------------------------------------
    Enable the outputs
    -------------------------------------------------------------------------*/
    CCER::setbit( timer->registers, bitMask );
  }


  void setCCOutputPolarity( Handle_rPtr timer, const Chimera::Timer::Output ch, const CCPolarity pol )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( ( ch >= Chimera::Timer::Output::NUM_OPTIONS ) || ( ( pol != CCP_OUT_ACTIVE_HIGH ) && ( pol != CCP_OUT_ACTIVE_LOW ) ) );

    /*-------------------------------------------------------------------------
    Set the appropriate enable flag bit
    -------------------------------------------------------------------------*/
    if( pol == CCP_OUT_ACTIVE_HIGH )
    {
      CCER::clear( timer->registers, PolarityFlags[ EnumValue( ch ) ] );
    }
    else
    {
      CCER::setbit( timer->registers, PolarityFlags[ EnumValue( ch ) ] );
    }
  }


  void setCCOutputPolarityBulk( Handle_rPtr timer, const uint32_t bf, const CCPolarity pol )
  {
    RT_DBG_ASSERT( ( pol != CCP_OUT_ACTIVE_HIGH ) && ( pol != CCP_OUT_ACTIVE_LOW ) );

    /*-------------------------------------------------------------------------
    Build up the bit mask
    -------------------------------------------------------------------------*/
    uint32_t bitMask = 0;
    for( size_t idx = 0; idx < PolarityFlags.size(); idx++ )
    {
      if( bf & ( 1u << idx ) )
      {
        bitMask |= PolarityFlags[ idx ];
      }
    }

    /*-------------------------------------------------------------------------
    Set the appropriate enable flag bits
    -------------------------------------------------------------------------*/
    if( pol == CCP_OUT_ACTIVE_HIGH )
    {
      CCER::clear( timer->registers, bitMask );
    }
    else
    {
      CCER::setbit( timer->registers, bitMask );
    }
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
