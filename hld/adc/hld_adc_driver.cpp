/********************************************************************************
 *  File Name:
 *    hld_adc_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor ADC interface.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>

/* Aurora Includes */
#include <Aurora/constants>

/* Chimera Includes */
#include <Chimera/adc>
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/adc>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/interrupt>

#if defined( THOR_HLD_ADC )

namespace Thor::ADC
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  namespace HLD = ::Thor::ADC;
  namespace LLD = ::Thor::LLD::ADC;

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_ADC_PERIPHS;
  static constexpr size_t NUM_ISR_SIG = LLD::NUM_ADC_IRQ_HANDLERS;

  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static size_t s_driver_initialized;                        /**< Tracks the module level initialization state */
  static HLD::Driver hld_driver[ NUM_DRIVERS ];              /**< Driver objects */

  //Should the LLD interface have an ISR event queue for each driver?
  // Each supported channel should have a queue right? The measurement is
  // taken off each channel, ISR fires, data retrieved, push to queue, signal
  // the userspace ISR handler to retrieve the data.

  // High resolution timestamps? Microseconds?


  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
  /**
   * @brief Userspace handler for interrupt events.
   * This handles all instances of ADC peripherals and all their interrupts.
   *
   * @param arg     ignored
   */
  static void ADCxISRUserThread( void *arg )
  {
    using namespace Chimera::Thread;

    while ( 1 )
    {
      /*-------------------------------------------------
      Wait for something to wake this thread. If the msg
      isn't correct, go back to waiting.
      -------------------------------------------------*/
      if( !this_thread::pendTaskMsg( ITCMsg::TSK_MSG_ISR_HANDLER ) )
      {
        continue;
      }

      /*-------------------------------------------------
      Handle every ISR. Don't know which triggered this.
      -------------------------------------------------*/
      for( size_t index = 0; index < NUM_DRIVERS; index++ )
      {
        hld_driver[ index ].postISRProcessing();
      }
    }
  }

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    using namespace Chimera::Thread;

    /*------------------------------------------------
    Prevent re-initialization from occurring
    ------------------------------------------------*/
    auto result = Chimera::Status::OK;
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return result;
    }
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    result = Thor::LLD::ADC::initialize();

    /*------------------------------------------------
    Register the ISR post processor thread
    ------------------------------------------------*/
    Task userThread;
    TaskConfig cfg;

    cfg.arg        = nullptr;
    cfg.function   = ADCxISRUserThread;
    cfg.priority   = Priority::MAXIMUM;
    cfg.stackWords = STACK_BYTES( 512 );
    cfg.type       = TaskInitType::DYNAMIC;
    cfg.name       = "PP_ADCx";

    userThread.create( cfg );
    Thor::LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_ADC, userThread.start() );

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }


  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }


  Driver_rPtr getDriver( const Chimera::ADC::Peripheral periph )
  {
    if ( auto idx = LLD::getResourceIndex( periph ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return &hld_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( Chimera::ADC::Peripheral::UNKNOWN )
  {
    mConfig.clear();
  }


  Driver::~Driver()
  {
  }


  void Driver::postISRProcessing()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::ADC::DriverConfig &cfg )
  {
    /*-------------------------------------------------
    Validate inputs. Currently all basic configuration
    options are supported by this driver.
    -------------------------------------------------*/
    auto driver           = LLD::getDriver( cfg.periph );
    auto lldResourceIndex = LLD::getResourceIndex( cfg.periph );

    if ( driver )
    {
      mConfig = cfg;
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------*/
    return driver->initialize( mConfig );
  }


  void Driver::close()
  {
    /*-------------------------------------------------
    Reset the hardware resources
    -------------------------------------------------*/
    auto lld = LLD::getDriver( mConfig.periph );
    lld->reset();
    lld->clockReset();
    lld->clockDisable();
  }


  Chimera::Status_t Driver::setSampleTime( const Chimera::ADC::Channel ch, const size_t cycles )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if ( !( ch < Chimera::ADC::Channel::NUM_OPTIONS ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Convert the sample times
    -------------------------------------------------*/
    auto driver = LLD::getDriver( mConfig.periph );
    if ( cycles < 3 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_2P5 );
    }
    else if ( cycles < 7 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_6P5 );
    }
    else if ( cycles < 13 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_12P5 );
    }
    else if ( cycles < 25 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_24P5 );
    }
    else if ( cycles < 48 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_47P5 );
    }
    else if ( cycles < 93 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_92P5 );
    }
    else if ( cycles < 248 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_247P5 );
    }
    else
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_640P5 );
    }
  }


  Chimera::ADC::Sample_t Driver::sampleChannel( const Chimera::ADC::Channel ch )
  {
    if ( !( ch < Chimera::ADC::Channel::NUM_OPTIONS ) )
    {
      return Chimera::ADC::INVALID_SAMPLE;
    }

    return LLD::getDriver( mConfig.periph )->sampleChannel( ch );
  }


  Chimera::Status_t Driver::configSequence( const Chimera::ADC::SequenceInit &cfg )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::startSequence()
  {

  }


  void Driver::stopSequence()
  {

  }


  void Driver::onInterrupt( const Chimera::ADC::Interrupt bmSignal, Chimera::ADC::ISRCallback cb )
  {
  }


  float Driver::sampleToVoltage( const Chimera::ADC::Sample_t sample )
  {
    return LLD::getDriver( mConfig.periph )->sampleToVoltage( sample );
  }

}    // namespace Thor::ADC

#endif /* THOR_HLD_ADC */
