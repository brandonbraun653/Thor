/********************************************************************************
 *  File Name:
 *    hld_timer_driver_basic.cpp
 *
 *  Description:
 *    Implementation of the Basic Timer HLD
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>
#include <memory>

/* Aurora Includes */
#include <Aurora/constants>

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
  static constexpr size_t NUM_PERIPHS = Thor::LLD::TIMER::NUM_BASIC_PERIPHS;

  /*-------------------------------------------------------------------------------
  Driver Memory
  -------------------------------------------------------------------------------*/
  std::array<BasicDriver_rPtr, NUM_PERIPHS> hld_basic_drivers;
  static std::array<LLD::IBasicDriver_rPtr, NUM_PERIPHS> s_lld_basic_drivers;

  /*-------------------------------------------------------------------------------
  Free Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initBasicDriverModule()
  {
    for ( size_t x = 0; x < NUM_PERIPHS; x++ )
    {
      hld_basic_drivers[ x ]   = nullptr;
      s_lld_basic_drivers[ x ] = nullptr;
    }

    return Chimera::Status::OK;
  }

  Chimera::Status_t initBasicDriverObject( const Thor::HLD::RIndex index )
  {
    if ( ( index.value() < hld_basic_drivers.size() ) && !hld_basic_drivers[ index.value() ] )
    {
      /* Initialize the HLD reference */
      auto driver       = std::make_shared<BasicDriver>();
      driver->mIndexHLD = index;

      /* Assign the driver instances */
      hld_basic_drivers[ index.value() ]   = driver;
      s_lld_basic_drivers[ index.value() ] = LLD::getBasicDriver( index );
    }

    return Chimera::Status::OK;
  }


  BasicDriver_rPtr getBasicDriver_rPtr( const Chimera::Timer::Peripheral periph, const bool create )
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
    auto pRegistered = Thor::LLD::TIMER::PeripheralToHLDResourceIndex.at( periph );

    /*------------------------------------------------
    Use the returned resource index to grab the driver instance
    ------------------------------------------------*/
    auto iDriver = pRegistered.second;
    if ( create )
    {
      initBasicDriverObject( iDriver );
    }

    return hld_basic_drivers[ iDriver.value() ];
  }

  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  Basic Driver Interface
  ------------------------------------------------*/
  BasicDriver::BasicDriver() : mIndexHLD( 0 ), mIndexLLD( 0 )
  {
  }

  BasicDriver::~BasicDriver()
  {
  }

  /*-------------------------------------------------
  Chimera ITimer Interface
  -------------------------------------------------*/
  Chimera::Status_t BasicDriver::initializeCoreFeature( const Chimera::Timer::CoreFeature feature,
                                                        Chimera::Timer::CoreFeatureInit &init )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t BasicDriver::invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t BasicDriver::setState( const Chimera::Timer::Switchable device,
                                           const Chimera::Timer::SwitchableState state )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t BasicDriver::requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  const Chimera::Timer::Descriptor *BasicDriver::getDeviceInfo()
  {
    return nullptr;
  }
}    // namespace Thor::TIMER

#endif /* THOR_HLD_TIMER */
