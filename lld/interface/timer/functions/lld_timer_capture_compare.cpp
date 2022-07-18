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
  /* clang-format off */
  static const std::array<uint32_t, EnumValue( Chimera::Timer::Output::NUM_OPTIONS )> s_enable_flags = {
    CCER_CC1E,  /* OUTPUT_1P */
    CCER_CC1NE, /* OUTPUT_1N */
    CCER_CC2E,
    CCER_CC2NE,
    CCER_CC3E,
    CCER_CC3NE,
    CCER_CC4E,
    0,          /* OUTPUT_4N */
    CCER_CC5E,
    0,          /* OUTPUT_5N */
    CCER_CC6E,
    0           /* OUTPUT_6N */
  };

  static const std::array<uint32_t, EnumValue( Chimera::Timer::Output::NUM_OPTIONS )> s_polarity_flags = {
    CCER_CC1P,  /* OUTPUT_1P */
    CCER_CC1NP, /* OUTPUT_1N */
    CCER_CC2P,
    CCER_CC2NP,
    CCER_CC3P,
    CCER_CC3NP,
    CCER_CC4P,
    CCER_CC4NP,
    CCER_CC5P,
    0,          /* OUTPUT_5N */
    CCER_CC6P,
    0           /* OUTPUT_6N */
  };

  static const std::array<uint32_t, EnumValue( Chimera::Timer::Output::NUM_OPTIONS )> s_idle_flags = {
    CR2_OIS1,   /* OUTPUT_1P */
    CR2_OIS1N,  /* OUTPUT_1N */
    CR2_OIS2,
    CR2_OIS2N,
    CR2_OIS3,
    CR2_OIS3N,
    CR2_OIS4,
    0,          /* OUTPUT_4N */
    CR2_OIS5,
    0,          /* OUTPUT_5N */
    CR2_OIS6,
    0           /* OUTPUT_6N */
  };
  /* clang-format on */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void setOutputIdleState( Handle_rPtr timer, const Chimera::Timer::Output ch, const Chimera::GPIO::State state )
  {
    if( state == Chimera::GPIO::State::HIGH )
    {
      CR2::set( timer->registers, s_idle_flags[ EnumValue( ch ) ] );
    }
    else
    {
      CR2::clear( timer->registers, s_idle_flags[ EnumValue( ch ) ] );
    }
  }


  Chimera::Status_t disableCCOutput( Handle_rPtr timer, const Chimera::Timer::Output ch )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
#if defined( DEBUG )
    if ( ch >= Chimera::Timer::Output::NUM_OPTIONS )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
#endif

    /*-------------------------------------------------------------------------
    Set the appropriate enable flag bit
    -------------------------------------------------------------------------*/
    CCER::clear( timer->registers, s_enable_flags[ EnumValue( ch ) ] );
    return Chimera::Status::OK;
  }


  Chimera::Status_t enableCCOutput( Handle_rPtr timer, const Chimera::Timer::Output ch )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
#if defined( DEBUG )
    if ( ch >= Chimera::Timer::Output::NUM_OPTIONS )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
#endif

    /*-------------------------------------------------------------------------
    Set the appropriate enable flag bit
    -------------------------------------------------------------------------*/
    CCER::setbit( timer->registers, s_enable_flags[ EnumValue( ch ) ] );
    return Chimera::Status::OK;
  }


  Chimera::Status_t setCCOutputPolarity( Handle_rPtr timer, const Chimera::Timer::Output ch, const CCPolarity pol )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
#if defined( DEBUG )
    if ( ( ch >= Chimera::Timer::Output::NUM_OPTIONS ) || ( ( pol != CCP_OUT_ACTIVE_HIGH ) && ( pol != CCP_OUT_ACTIVE_LOW ) ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
#endif

    /*-------------------------------------------------------------------------
    Set the appropriate enable flag bit
    -------------------------------------------------------------------------*/
    if( pol == CCP_OUT_ACTIVE_HIGH )
    {
      CCER::clear( timer->registers, s_polarity_flags[ EnumValue( ch ) ] );
    }
    else
    {
      CCER::setbit( timer->registers, s_polarity_flags[ EnumValue( ch ) ] );
    }

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
