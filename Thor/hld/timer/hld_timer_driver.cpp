/********************************************************************************
 *  File Name:
 *    hld_timer_driver.cpp
 *
 *  Description:
 *    Driver implementation 
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>
#include <Thor/lld/interface/timer/timer_intf.hpp>


#if defined( THOR_HLD_TIMER )

namespace Thor::Timer
{
  Chimera::Status_t initialize()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t reset()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  void incrementSystemTick()
  {
    Thor::LLD::TIMER::incrementSystemTick();
  }

  size_t millis()
  {
    return Thor::LLD::TIMER::millis();
  }

  void delayMilliseconds( const size_t ms )
  {
    Thor::LLD::TIMER::delayMilliseconds( ms );
  }

  void delayMicroseconds( const size_t us )
  {
    Thor::LLD::TIMER::delayMicroseconds( us );
  }

  /*------------------------------------------------
  Driver Implementation
  ------------------------------------------------*/
  Driver::Driver()
  {

  }

  Driver::~Driver()
  {

  }

  void Driver::enable()
  {
    
  }

  void Driver::disable()
  {
    
  }

  Chimera::Status_t Driver::initPeripheral( const Chimera::Timer::DriverConfig &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::enableISR( const Chimera::Timer::ISREvent type )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::disableISR( const Chimera::Timer::ISREvent type )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::setupInputCapture( const Chimera::Timer::InputCapture::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::setupOutputCompare( const Chimera::Timer::OutputCompare::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::setupPWMGeneration( const Chimera::Timer::PWMGeneration::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::setupOnePulse( const Chimera::Timer::OnePulse::Config &cfg )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t Driver::counterBitWidth()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t Driver::tickRate( const Chimera::Units::Time units )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t Driver::maxPeriod( const Chimera::Units::Time units )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t Driver::minPeriod( const Chimera::Units::Time units )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Timer::Mode Driver::currentMode()
  {
    return Chimera::Timer::Mode::NUM_OPTIONS;
  }

  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}

#endif /* THOR_HLD_TIMER */
