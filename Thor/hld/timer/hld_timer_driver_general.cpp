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
      auto driver            = new GeneralDriver;
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

  /*------------------------------------------------
  Timer Base Interface
  ------------------------------------------------*/
  Chimera::Status_t GeneralDriver::initPeripheral( const Chimera::Timer::DriverConfig &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  bool GeneralDriver::configured()
  {
    return 0;
  }

  size_t GeneralDriver::counterBitWidth()
  {
    return 0;
  }

  size_t GeneralDriver::tickRate( const Chimera::Units::Time units )
  {
    return 0;
  }

  size_t GeneralDriver::maxPeriod( const Chimera::Units::Time units )
  {
    return 0;
  }

  size_t GeneralDriver::minPeriod( const Chimera::Units::Time units )
  {
    return 0;
  }

  bool GeneralDriver::hasFunction( const Chimera::Timer::Function func )
  {
    return false;
  }

  /*------------------------------------------------
  Timer Channel Interface
  ------------------------------------------------*/
  Chimera::Status_t GeneralDriver::enable( const Chimera::Timer::Channel channel )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GeneralDriver::disable( const Chimera::Timer::Channel channel )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GeneralDriver::enableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GeneralDriver::disableEvent( const Chimera::Timer::Channel channel, const Chimera::Timer::Event type )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Encoder Interface
  ------------------------------------------------*/
  Chimera::Status_t GeneralDriver::encInit( const Chimera::Timer::Encoder::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Input Capture Interface
  ------------------------------------------------*/
  Chimera::Status_t GeneralDriver::icInit( const Chimera::Timer::InputCapture::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  One Pulse Interface
  ------------------------------------------------*/
  Chimera::Status_t GeneralDriver::opInit( const Chimera::Timer::OnePulse::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Output Compare Interface
  ------------------------------------------------*/
  Chimera::Status_t GeneralDriver::ocInit( const Chimera::Timer::OutputCompare::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  PWM Interface
  ------------------------------------------------*/
  Chimera::Status_t GeneralDriver::pwmInit( const Chimera::Timer::PWM::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}  // namespace Thor::TIMER

#endif /* THOR_HLD_TIMER */
