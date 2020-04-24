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
  Driver Memory
  -------------------------------------------------------------------------------*/
  std::array<GeneralDriver_sPtr, NUM_PERIPHS> hld_general_drivers;
  static std::array<LLD::GeneralDriver_rPtr, NUM_PERIPHS> s_lld_general_drivers;

  /*-------------------------------------------------------------------------------
  Free Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initGeneralDriverModule()
  {
    for ( size_t x = 0; x < NUM_PERIPHS; x++ )
    {
      hld_general_drivers[ x ]   = nullptr;
      s_lld_general_drivers[ x ] = nullptr;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t initGeneralDriverObject( const Thor::HLD::RIndex index )
  {
    if ( ( index.value() < hld_general_drivers.size() ) && !hld_general_drivers[ index.value() ] )
    {
      /* Initialize the HLD reference */
      auto driver       = std::make_shared<GeneralDriver>();
      driver->mIndexHLD = index;

      /* Assign the driver instances */
      hld_general_drivers[ index.value() ]   = driver;
      s_lld_general_drivers[ index.value() ] = LLD::getGeneralDriver( index );
    }

    return Chimera::CommonStatusCodes::OK;
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

    return hld_general_drivers[ iDriver.value() ];
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
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
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
    auto driver     = s_lld_general_drivers[ mIndexHLD.value() ];

    driver->attach( peripheral );

    /*-------------------------------------------------
    Configure the base timer settings
    -------------------------------------------------*/
    driver->initBaseTimer( cfg );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t GeneralDriver::initPWM( const Chimera::Timer::PWM::Config &cfg )
  {
    /*------------------------------------------------
    Make sure we should actually be initializing the channel
    ------------------------------------------------*/
    if ( !cfg.validity )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    return s_lld_general_drivers[ mIndexHLD.value() ]->initPWM( cfg );
  }
}    // namespace Thor::TIMER

#endif /* THOR_HLD_TIMER */
