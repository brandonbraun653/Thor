/********************************************************************************
 *  File Name:
 *    hw_timer_driver_stm32l4_general.cpp
 *
 *  Description:
 *    LLD General Timer Driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_driver.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_mapping.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>

#if defined( THOR_LLD_TIMER )

namespace Thor::LLD::TIMER
{
  static std::array<GeneralDriver_rPtr, NUM_GENERAL_PERIPHS> s_general_drivers;

  /*-------------------------------------------------------------------------------
  LLD Public Free Functions
  -------------------------------------------------------------------------------*/
  GeneralDriver_rPtr getGeneralDriver( const Thor::HLD::RIndex channel )
  {
    const auto ch = channel.value();

    /*------------------------------------------------
    Input protection
    ------------------------------------------------*/
    if ( ch >= s_general_drivers.size() )
    {
      return nullptr;
    }

    /*------------------------------------------------
    Create the driver if it doesn't exist yet
    ------------------------------------------------*/
    if ( !s_general_drivers[ ch ] )
    {
      s_general_drivers[ ch ] = new GeneralDriverImpl;
    }

    return s_general_drivers[ ch ];
  }

  /*-------------------------------------------------------------------------------
  General Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  GeneralDriverImpl::GeneralDriverImpl() : mpPeriph( nullptr ), mRIndex( 0 )
  {
  }

  GeneralDriverImpl::~GeneralDriverImpl()
  {
  }

  Chimera::Status_t GeneralDriverImpl::reset()
  {
    return Chimera::Status::OK;
  }

  void GeneralDriverImpl::clockEnable()
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_TIM, mRIndex.value() );
  }

  void GeneralDriverImpl::clockDisable()
  {
    auto rcc = Thor::LLD::RCC::getPeriphClockCtrl();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_TIM, mRIndex.value() );
  }

  void GeneralDriverImpl::toggleCounter( const bool state )
  {
    state ? CEN::set( mpPeriph, TIM_CR1_CEN ) : CEN::set( mpPeriph, 0 );
  }

  void GeneralDriverImpl::toggleChannel( const Chimera::Timer::Channel channel, const bool state )
  {
    switch ( channel )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        state ? CC1E::set( mpPeriph, TIM_CCER_CC1E ) : CC1E::set( mpPeriph, 0 );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        state ? CC2E::set( mpPeriph, TIM_CCER_CC2E ) : CC2E::set( mpPeriph, 0 );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        state ? CC3E::set( mpPeriph, TIM_CCER_CC3E ) : CC3E::set( mpPeriph, 0 );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        state ? CC4E::set( mpPeriph, TIM_CCER_CC4E ) : CC4E::set( mpPeriph, 0 );
        break;

      default:
        break;
    }
  }

  Chimera::Status_t GeneralDriverImpl::attach( RegisterMap *const peripheral )
  {
    mpPeriph = peripheral;
    mRIndex  = RIndex( InstanceToResourceIndex.at( reinterpret_cast<std::uintptr_t>( peripheral ) ).second );

    return Chimera::Status::OK;
  }

  Chimera::Status_t GeneralDriverImpl::initBaseTimer( const Chimera::Timer::DriverConfig &cfg )
  {
    /*------------------------------------------------
    Turn on the peripehral clock and ensure the counter has been turned off
    ------------------------------------------------*/
    clockEnable();
    toggleCounter( false );

    /*------------------------------------------------
    Configure the base timer
    ------------------------------------------------*/
    /* Set the counter direction */
    DIR::set( mpPeriph, LUT_Direction[ static_cast<size_t>( cfg.countDirection ) ] );

    /* Set the Auto-Reload Register */
    ARR::set( mpPeriph, cfg.reloadValue );

    /* Automatically buffer the ARR register updates */
    ARPE::set( mpPeriph, TIM_CR1_ARPE );

    /* Set the clock prescaler */
    PSC::set( mpPeriph, cfg.prescaler );

    /* Reset the counter to the reload value, indicating a "reset" */
    CNT::set( mpPeriph, cfg.reloadValue );

    /* Make sure One Pulse mode is disabled */
    OPM::set( mpPeriph, 0 );

    /*------------------------------------------------
    Re-enable the counter
    ------------------------------------------------*/
    toggleCounter( true );
    return Chimera::Status::OK;
  }

  Chimera::Status_t GeneralDriverImpl::initPWM( const Chimera::Timer::PWM::Config &cfg )
  {
    using namespace Chimera::Timer;

    /*------------------------------------------------
    Only edge aligned mode is supported at the moment
    ------------------------------------------------*/
    if ( cfg.mode != PWM::Mode::EDGE_ALIGNED )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*------------------------------------------------
    Configure the given channel for PWM
    ------------------------------------------------*/
    /* Set the mode to default as PWM Mode 1 */
    setOutputCompareMode( cfg.outputChannel, Configuration::OC::Mode::PWM_MODE_1 );

    /* Enable the preload register */
    setOutputComparePreload( cfg.outputChannel, Configuration::ENABLED );

    /* Set the polarity */
    if ( cfg.polarity == PWM::Polarity::ACTIVE_HIGH )
    {
      setCaptureComparePolarity( cfg.outputChannel, Configuration::CC::Polarity::ACTIVE_HIGH );
    }
    else
    {
      setCaptureComparePolarity( cfg.outputChannel, Configuration::CC::Polarity::ACTIVE_LOW );
    }

    /* Set the channel direction to be configured as an output */
    setCaptureCompareDirection( cfg.outputChannel, Configuration::CC::Direction::OUTPUT );

    /* Enable the output */
    toggleChannel( cfg.outputChannel, true );

    /* Set the output compare match value */
    setCaptureCompareMatch( cfg.outputChannel, cfg.compareMatch );

    return Chimera::Status::OK;
  }


  void GeneralDriverImpl::setCaptureComparePolarity( const Chimera::Timer::Channel channel, const Reg32_t val )
  {
    switch ( channel )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        CC1P::set( mpPeriph, val << TIM_CCER_CC1P_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        CC2P::set( mpPeriph, val << TIM_CCER_CC2P_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        CC3P::set( mpPeriph, val << TIM_CCER_CC3P_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        CC4P::set( mpPeriph, val << TIM_CCER_CC4P_Pos );
        break;

      default:
        break;
    }
  }

  void GeneralDriverImpl::setCaptureCompareMatch( const Chimera::Timer::Channel channel, const Reg32_t val )
  {
    switch ( channel )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        CCR1::set( mpPeriph, val );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        CCR2::set( mpPeriph, val );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        CCR3::set( mpPeriph, val );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        CCR4::set( mpPeriph, val );
        break;

      default:
        break;
    }
  }

  void GeneralDriverImpl::setCaptureCompareDirection( const Chimera::Timer::Channel channel, const Reg32_t val )
  {
    switch ( channel )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        CC1S::set( mpPeriph, val << TIM_CCMR1_CC1S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        CC2S::set( mpPeriph, val << TIM_CCMR1_CC2S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        CC3S::set( mpPeriph, val << TIM_CCMR2_CC3S_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        CC4S::set( mpPeriph, val << TIM_CCMR2_CC4S_Pos );
        break;

      default:
        break;
    }
  }


  void GeneralDriverImpl::setOutputCompareMode( const Chimera::Timer::Channel channel, const Reg32_t val )
  {
    switch ( channel )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        OC1M::set( mpPeriph, val << TIM_CCMR1_OC1M_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        OC2M::set( mpPeriph, val << TIM_CCMR1_OC2M_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        OC3M::set( mpPeriph, val << TIM_CCMR2_OC3M_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        OC4M::set( mpPeriph, val << TIM_CCMR2_OC4M_Pos );
        break;

      default:
        break;
    }
  }

  void GeneralDriverImpl::setOutputComparePreload( const Chimera::Timer::Channel channel, const Reg32_t val )
  {
    switch ( channel )
    {
      case Chimera::Timer::Channel::CHANNEL_1:
        OC1PE::set( mpPeriph, val << TIM_CCMR1_OC1PE_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_2:
        OC2PE::set( mpPeriph, val << TIM_CCMR1_OC2PE_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_3:
        OC3PE::set( mpPeriph, val << TIM_CCMR2_OC3PE_Pos );
        break;

      case Chimera::Timer::Channel::CHANNEL_4:
        OC4PE::set( mpPeriph, val << TIM_CCMR2_OC4PE_Pos );
        break;

      default:
        break;
    }
  }

}    // namespace Thor::LLD::TIMER

#endif