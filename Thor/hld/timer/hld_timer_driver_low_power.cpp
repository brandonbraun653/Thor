/********************************************************************************
 *  File Name:
 *    hld_timer_driver_low_power.cpp
 *
 *  Description:
 *    Implements the Low Power Timer HLD
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
  namespace LLD = Thor::LLD::TIMER;
  static constexpr size_t NUM_PERIPHS = Thor::LLD::TIMER::NUM_LOW_POWER_PERIPHS;

  /*-------------------------------------------------------------------------------
  Driver Memory
  -------------------------------------------------------------------------------*/
  std::array<LowPowerDriver_sPtr, NUM_PERIPHS> hld_low_power_drivers;
  static std::array<LLD::ILowPowerDriver_sPtr, NUM_PERIPHS> s_lld_low_power_drivers;

  /*-------------------------------------------------------------------------------
  Free Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initializeLowPower()
  {
    for( size_t x=0; x<NUM_PERIPHS; x++)
    {
      hld_low_power_drivers[ x ] = nullptr;
      s_lld_low_power_drivers[ x ] = nullptr;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  LowPowerDriver_sPtr getLowPowerDriver_sPtr( const Chimera::Timer::Peripheral periph, const bool create )
  {
    /*-------------------------------------------------
    Ensure the driver is initialized. This saves the cost
    of a look-up in the next step.
    -------------------------------------------------*/
    if( !isInitialized() )
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
    if ( !hld_low_power_drivers[ iDriver ] && create )
    {
      /* Initialize the HLD reference */
      auto driver            = std::make_shared<LowPowerDriver>();
      driver->mResourceIndex = iDriver;

      hld_low_power_drivers[ iDriver ] = driver;

      /* Initialize the LLD reference */
      s_lld_low_power_drivers[ iDriver ] = LLD::getLowPowerDriver( iDriver );
    }

    return hld_low_power_drivers[ iDriver ];
  }

  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  Low Power Driver Interface
  ------------------------------------------------*/
  LowPowerDriver::LowPowerDriver() : mResourceIndex( 0 )
  {
  }

  LowPowerDriver::~LowPowerDriver()
  {
  }

  /*-------------------------------------------------
  Chimera ITimer Interface
  -------------------------------------------------*/
  Chimera::Status_t LowPowerDriver::initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t LowPowerDriver::invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t LowPowerDriver::setState( const Chimera::Timer::Switchable device,
                                              const Chimera::Timer::SwitchableState state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t LowPowerDriver::requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  const Chimera::Timer::Descriptor *LowPowerDriver::getDeviceInfo()
  {
    return nullptr;
  }
}  // namespace Thor::TIMER

#endif /* THOR_HLD_TIMER */
