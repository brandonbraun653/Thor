/********************************************************************************
 *  File Name:
 *    hld_timer_driver_general.cpp
 *
 *  Description:
 *    Implementation of the General Timer HLD
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>
#include <Thor/hld/timer/hld_timer_prv_driver.hpp>
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/interface/timer/timer_detail.hpp>


#if defined( THOR_HLD_TIMER )

namespace Thor::TIMER
{
  namespace LLD                       = Thor::LLD::TIMER;
  static constexpr size_t NUM_PERIPHS = Thor::LLD::TIMER::NUM_GENERAL_PERIPHS;

  /*-------------------------------------------------------------------------------
  Types
  -------------------------------------------------------------------------------*/
  struct ChannelConfiguration
  {
    Chimera::Timer::CoreFeature configuredAs;
    Chimera::Timer::CoreFeatureInit configData;
  };

  struct GeneralTimerData
  {
    GeneralDriver_sPtr hld_driver;
    LLD::GeneralDriver_rPtr lld_driver;

    Chimera::Timer::DriverConfig baseTimerConfig;
    std::array<ChannelConfiguration, NUM_PERIPHS> channelConfig;
  };

  /*-------------------------------------------------------------------------------
  Driver Memory
  -------------------------------------------------------------------------------*/
  static std::array<GeneralTimerData, NUM_PERIPHS> s_prv_timer_data;

  /*-------------------------------------------------------------------------------
  Free Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initGeneralDriverModule()
  {
    s_prv_timer_data.fill( {} );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t initGeneralDriverObject( const Thor::HLD::RIndex index )
  {
    if ( ( index.value() < s_prv_timer_data.size() ) && !s_prv_timer_data[ index.value() ].hld_driver )
    {
      /* Initialize the HLD reference */
      auto driver       = std::make_shared<GeneralDriver>();
      driver->mIndexHLD = index;

      /* Assign the driver instances */
      s_prv_timer_data[ index.value() ].hld_driver = driver;
      s_prv_timer_data[ index.value() ].lld_driver = LLD::getGeneralDriver( index );
    }

    return Chimera::CommonStatusCodes::OK;
  }

  GeneralDriver_sPtr getGeneralDriverObject( const Thor::HLD::RIndex index )
  {
    if ( index.value() < NUM_PERIPHS ) 
    {
      return s_prv_timer_data[ index.value() ].hld_driver;
    }

    return nullptr;
  }

  GeneralDriver_sPtr getGeneralDriver_sPtr( const Chimera::Timer::Peripheral periph, const bool create )
  {
    /*-------------------------------------------------
    Ensure the driver is initialized. This saves the cost
    of a look-up in the next step.
    -------------------------------------------------*/
    if ( !isInitialized() )
    {
      return nullptr;
    }

    /*------------------------------------------------
    Check to see if the peripheral is supported by the LLD
    ------------------------------------------------*/
    auto pRegistered = Thor::LLD::TIMER::PeripheralToHLDResourceIndex.find( periph );
    if ( !pRegistered )
    {
      return nullptr;
    }

    /*------------------------------------------------
    Use the returned resource index to grab the driver instance
    ------------------------------------------------*/
    auto const iDriver = pRegistered->second;
    if ( create )
    {
      initGeneralDriverObject( iDriver );
    }

    return s_prv_timer_data[ iDriver.value() ].hld_driver;
  }

  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Chimera ITimer Interface
  -------------------------------------------------*/
  Chimera::Status_t GeneralDriver::initializeCoreFeature( const Chimera::Timer::CoreFeature feature,
                                                          Chimera::Timer::CoreFeatureInit &init )
  {
    using namespace Chimera::Timer;

    switch ( feature )
    {
      case CoreFeature::BASE_TIMER:
        return initCoreTimer( init.base );
        break;

      case CoreFeature::PWM_OUTPUT:
        return initPWM( init.pwm );

      default:
        return Chimera::CommonStatusCodes::NOT_SUPPORTED;
        break;
    }
  }

  Chimera::Status_t GeneralDriver::invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize )
  {
    using namespace Chimera::Timer;

    switch ( action )
    {
      case DriverAction::ENABLE_PWM_CHANNEL:
      case DriverAction::DISABLE_PWM_CHANNEL:
      {
        /* Ensure the parameter spec was met */
        if ( !arg || ( argSize != sizeof( Channel ) ) ) 
        {
          return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
        }

        /* Convert the arguments to the expected types */
        bool state = ( action == DriverAction::ENABLE_PWM_CHANNEL );
        auto channel = *static_cast<Channel *>( arg );

        /* Invoke the desired action */
        return toggleOutput( channel, state );
      }
      break;

      default:
        return Chimera::CommonStatusCodes::NOT_SUPPORTED;
        break;
    }
  }

  Chimera::Status_t GeneralDriver::setState( const Chimera::Timer::Switchable device,
                                             const Chimera::Timer::SwitchableState state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GeneralDriver::requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  const Chimera::Timer::Descriptor *GeneralDriver::getDeviceInfo()
  {
    return nullptr;
  }


  /*------------------------------------------------
  General Driver Interface
  ------------------------------------------------*/
  GeneralDriver::GeneralDriver() : mIndexHLD( 0 ), mIndexLLD( 0 )
  {
  }

  GeneralDriver::~GeneralDriver()
  {
  }

  Chimera::Status_t GeneralDriver::initCoreTimer( const Chimera::Timer::DriverConfig &cfg )
  {
    /*------------------------------------------------
    Make sure we should actually be initializing the timer
    ------------------------------------------------*/
    if ( !cfg.validity )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Look up the LLD resource index and use it to attach the proper peripheral instance
    -------------------------------------------------*/
    mIndexLLD       = LLD::PeripheralToLLDResourceIndex.find( cfg.peripheral )->second;
    auto peripheral = reinterpret_cast<LLD::RegisterMap *>( LLD::LUT_PeripheralList[ mIndexLLD.value() ] );
    auto driver     = s_prv_timer_data[ mIndexHLD.value() ].lld_driver;
    auto result     = Chimera::CommonStatusCodes::OK;
    
    /*-------------------------------------------------
    Configure the base timer settings
    -------------------------------------------------*/
    result |= driver->attach( peripheral );
    result |= driver->initBaseTimer( cfg );

    if ( result == Chimera::CommonStatusCodes::OK ) 
    {
      s_prv_timer_data[ mIndexHLD.value() ].baseTimerConfig = cfg;
    }

    return result;
  }

  Chimera::Status_t GeneralDriver::initPWM( const Chimera::Timer::PWM::Config &cfg )
  {
    using namespace Chimera::Timer;

    /*------------------------------------------------
    Make sure we should actually be initializing the channel
    ------------------------------------------------*/
    if ( !cfg.validity )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Configure the PWM settings
    -------------------------------------------------*/
    auto result = s_prv_timer_data[ mIndexHLD.value() ].lld_driver->initPWM( cfg );

    if ( result == Chimera::CommonStatusCodes::OK ) 
    {
      auto index = mIndexHLD.value();
      auto channel = static_cast<size_t>( cfg.outputChannel );

      s_prv_timer_data[ index ].channelConfig[ channel ].configuredAs   = CoreFeature::PWM_OUTPUT;
      s_prv_timer_data[ index ].channelConfig[ channel ].configData.pwm = cfg;
    }

    return result;
  }

  Chimera::Status_t GeneralDriver::toggleOutput( const Chimera::Timer::Channel channel, const bool state )
  {
    s_prv_timer_data[ mIndexHLD.value() ].lld_driver->toggleChannel( channel, state );
    return Chimera::CommonStatusCodes::OK;
  }
}    // namespace Thor::TIMER

#endif /* THOR_HLD_TIMER */
