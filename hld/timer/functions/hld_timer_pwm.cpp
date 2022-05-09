/******************************************************************************
 *  File Name:
 *    hld_timer_pwm.cpp
 *
 *  Description:
 *    Thor implementation of the Chimera PWM timer driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/peripheral>
#include <Chimera/timer>
#include <Thor/lld/interface/inc/timer>

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
static constexpr size_t REQ_HW_TIMER_TYPES =
    ( Thor::LLD::TIMER::HardwareType::TIMER_HW_ADVANCED | Thor::LLD::TIMER::HardwareType::TIMER_HW_BASIC |
      Thor::LLD::TIMER::HardwareType::TIMER_HW_GENERAL );

/*-----------------------------------------------------------------------------
Structures
-----------------------------------------------------------------------------*/
/**
 * @brief PWM controller data
 */
struct ControlBlock
{

};

/*-----------------------------------------------------------------------------
Static Data
-----------------------------------------------------------------------------*/
static Chimera::DeviceManager<ControlBlock, Chimera::Timer::Instance,
                              EnumValue( Chimera::Timer::Instance::NUM_OPTIONS )>
    s_driver_resources;


namespace Chimera::Timer::PWM
{
  Driver::Driver() : mTimerImpl( nullptr )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init( const DriverConfig &cfg )
  {
    using namespace Thor::LLD::TIMER;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !( getHardwareType( cfg.coreCfg.instance ) & REQ_HW_TIMER_TYPES ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Create the driver handle if it hasn't been already
    -------------------------------------------------------------------------*/
    if( !mTimerImpl )
    {
      mTimerImpl = std::make_shared<void *>();


      // Great idea!
      // Use the hash of the configuration to assign a key to void pointer?
    }

    /*-------------------------------------------------------------------------
    Grab the driver for this instance and register it with the class
    -------------------------------------------------------------------------*/
    auto driver = s_driver_resources.getOrCreate( cfg.coreCfg.instance );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::enableOutput()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::disableOutput()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setFrequency( const size_t freq )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setDutyCycle( const size_t dutyCycle )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setPolarity( const Chimera::Timer::PWM::Polarity polarity )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

}    // namespace Chimera::Timer::PWM
