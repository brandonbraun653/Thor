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
  Chimera::Status_t initializeAdvanced()
  {
    for ( size_t x = 0; x < NUM_PERIPHS; x++ )
    {
      hld_advanced_drivers[ x ]   = nullptr;
      s_lld_advanced_drivers[ x ] = nullptr;
    }

    return Chimera::CommonStatusCodes::OK;
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
    auto pRegistered = Thor::LLD::TIMER::PeripheralToHLDResourceIndex.find( periph );
    if ( !pRegistered )
    {
      return nullptr;
    }

    /*------------------------------------------------
    Use the returned resource index to grab the driver instance
    ------------------------------------------------*/
    auto const iDriver = pRegistered->second;
    if ( !hld_advanced_drivers[ iDriver ] && create )
    {
      /* Initialize the HLD reference */
      auto driver            = new AdvancedDriver;
      driver->mResourceIndex = iDriver;

      hld_advanced_drivers[ iDriver ] = driver;

      /* Initialize the LLD reference */
      s_lld_advanced_drivers[ iDriver ] = LLD::getAdvancedDriver( iDriver );
    }

    return hld_advanced_drivers[ iDriver ];
  }

  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  Advanced Driver Interface
  ------------------------------------------------*/
  AdvancedDriver::AdvancedDriver() : mResourceIndex( 0 )
  {
  }

  AdvancedDriver::~AdvancedDriver()
  {
  }

  /*------------------------------------------------
  Timer Base Interface
  ------------------------------------------------*/
  Chimera::Status_t AdvancedDriver::initPeripheral( const Chimera::Timer::DriverConfig &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  bool AdvancedDriver::configured()
  {
    return 0;
  }

  size_t AdvancedDriver::counterBitWidth()
  {
    return 0;
  }

  size_t AdvancedDriver::tickRate( const Chimera::Units::Time units )
  {
    return 0;
  }

  size_t AdvancedDriver::maxPeriod( const Chimera::Units::Time units )
  {
    return 0;
  }

  size_t AdvancedDriver::minPeriod( const Chimera::Units::Time units )
  {
    return 0;
  }

  bool AdvancedDriver::hasFunction( const Chimera::Timer::Function func )
  {
    return false;
  }

  /*------------------------------------------------
  Timer Channel Interface
  ------------------------------------------------*/
  Chimera::Status_t AdvancedDriver::enable( const Chimera::Timer::Channel channel )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t AdvancedDriver::disable( const Chimera::Timer::Channel channel )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t AdvancedDriver::enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t AdvancedDriver::disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Encoder Interface
  ------------------------------------------------*/
  Chimera::Status_t AdvancedDriver::encInit( const Chimera::Timer::Encoder::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Input Capture Interface
  ------------------------------------------------*/
  Chimera::Status_t AdvancedDriver::icInit( const Chimera::Timer::InputCapture::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  One Pulse Interface
  ------------------------------------------------*/
  Chimera::Status_t AdvancedDriver::opInit( const Chimera::Timer::OnePulse::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Output Compare Interface
  ------------------------------------------------*/
  Chimera::Status_t AdvancedDriver::ocInit( const Chimera::Timer::OutputCompare::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  PWM Interface
  ------------------------------------------------*/
  Chimera::Status_t AdvancedDriver::pwmInit( const Chimera::Timer::PWM::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}    // namespace Thor::TIMER

#endif /* THOR_HLD_TIMER */
