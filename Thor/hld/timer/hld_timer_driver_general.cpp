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
  namespace LLD = Thor::LLD::TIMER;
  static constexpr size_t NUM_PERIPHS = Thor::LLD::TIMER::NUM_GENERAL_PERIPHS;

  /*-------------------------------------------------------------------------------
  Driver Memory
  -------------------------------------------------------------------------------*/
  std::array<GeneralDriver_sPtr, NUM_PERIPHS> hld_general_drivers;
  static std::array<LLD::IGeneralDriver_sPtr, NUM_PERIPHS> s_lld_general_drivers;

  /*-------------------------------------------------------------------------------
  Free Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initializeGeneral()
  {
    for( size_t x=0; x<NUM_PERIPHS; x++)
    {
      hld_general_drivers[ x ] = nullptr;
      s_lld_general_drivers[ x ] = nullptr;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  GeneralDriver_sPtr getGeneralDriver_sPtr( const Chimera::Timer::Peripheral periph, const bool create )
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
    if ( !hld_general_drivers[ iDriver ] && create )
    {
      /* Initialize the HLD reference */
      auto driver            = std::make_shared<GeneralDriver>();
      driver->mResourceIndex = iDriver;

      hld_general_drivers[ iDriver ] = driver;

      /* Initialize the LLD reference */
      s_lld_general_drivers[ iDriver ] = LLD::getGeneralDriver( iDriver );
    }

    return hld_general_drivers[ iDriver ];
  }

  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  General Driver Interface
  ------------------------------------------------*/
  GeneralDriver::GeneralDriver() : mResourceIndex( 0 )
  {
  }

  GeneralDriver::~GeneralDriver()
  {
  }

  /*-------------------------------------------------
  Chimera ITimer Interface
  -------------------------------------------------*/
  Chimera::Status_t GeneralDriver::initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
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
}  // namespace Thor::TIMER

#endif /* THOR_HLD_TIMER */
