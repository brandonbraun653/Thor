/********************************************************************************
 *  File Name:
 *    hld_timer_driver_advanced.cpp
 *
 *  Description:
 *    Implemenation for the Advanced Timer HLD
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
  static constexpr size_t NUM_PERIPHS = Thor::LLD::TIMER::NUM_ADVANCED_PERIPHS;

  /*-------------------------------------------------------------------------------
  Driver Memory
  -------------------------------------------------------------------------------*/
  std::array<AdvancedDriver_sPtr, NUM_PERIPHS> hld_advanced_drivers;
  static std::array<LLD::IAdvancedDriver_sPtr, NUM_PERIPHS> s_lld_advanced_drivers;

  /*-------------------------------------------------------------------------------
  Free Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initAdvancedDriverModule()
  {
    for ( size_t x = 0; x < NUM_PERIPHS; x++ )
    {
      hld_advanced_drivers[ x ]   = nullptr;
      s_lld_advanced_drivers[ x ] = nullptr;
    }

    return Chimera::Status::OK;
  }

  Chimera::Status_t initAdvancedDriverObject( const Thor::HLD::RIndex index )
  {
    if ( ( index.value() < hld_advanced_drivers.size() ) && !hld_advanced_drivers[ index.value() ] )
    {
      /* Initialize the HLD reference */
      auto driver            = std::make_shared<AdvancedDriver>();
      driver->mIndexHLD = index;

      /* Assign the driver instances */
      hld_advanced_drivers[ index.value() ]   = driver;
      s_lld_advanced_drivers[ index.value() ] = LLD::getAdvancedDriver( index );
    }

    return Chimera::Status::OK;
  }

  AdvancedDriver_sPtr getAdvancedDriver_sPtr( const Chimera::Timer::Peripheral periph, const bool create )
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
    const auto pRegistered = LLD::PeripheralToHLDResourceIndex.at( periph );

    /*------------------------------------------------
    Use the returned resource index to grab the driver instance
    ------------------------------------------------*/
    auto const iDriver = pRegistered.second;
    if ( create )
    {
      initAdvancedDriverObject( iDriver );
    }

    return hld_advanced_drivers[ iDriver.value() ];
  }

  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  Advanced Driver Interface
  ------------------------------------------------*/
  AdvancedDriver::AdvancedDriver() : mIndexHLD( 0 ), mIndexLLD( 0 )
  {
  }

  AdvancedDriver::~AdvancedDriver()
  {
  }

  /*-------------------------------------------------
  Chimera ITimer Interface
  -------------------------------------------------*/
  Chimera::Status_t AdvancedDriver::initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t AdvancedDriver::invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t AdvancedDriver::setState( const Chimera::Timer::Switchable device,
                                              const Chimera::Timer::SwitchableState state )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t AdvancedDriver::requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  const Chimera::Timer::Descriptor *AdvancedDriver::getDeviceInfo()
  {
    return nullptr;
  }
}    // namespace Thor::TIMER

#endif /* THOR_HLD_TIMER */
