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
#include <bitset>
#include <cstring>

/* Aurora Includes */
#include <Aurora/constants>
#include <Aurora/logging>
#include <Aurora/utility>

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
  static size_t s_driver_initialized;           /**< Tracks the module level initialization state */
  static HLD::Driver hld_driver[ NUM_DRIVERS ]; /**< Driver objects */


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
    using namespace Aurora::Logging;
    using namespace Chimera::Thread;

    TaskMsg msg;

    while ( 1 )
    {
      /*-------------------------------------------------
      Wait for something to wake this thread. If the msg
      isn't correct, go back to waiting.
      -------------------------------------------------*/
      if ( !this_thread::receiveTaskMsg( msg, TIMEOUT_BLOCK ) )
      {
        continue;
      }

      /*-------------------------------------------------
      Handle every ISR. Don't know which triggered this.
      -------------------------------------------------*/
      if( msg & ITCMsg::TSK_MSG_ISR_DATA_READY )
      {
        for ( size_t index = 0; index < NUM_DRIVERS; index++ )
        {
          hld_driver[ index ].postISRProcessing();
        }
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
    cfg.stackWords = STACK_BYTES( 4096 );
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
    using namespace Chimera::ADC;

    for( size_t ch = 0; ch < Thor::LLD::ADC::NUM_ADC_CHANNELS_PER_PERIPH; ch++ )
    {
      /*-------------------------------------------------
      Populate the interrupt information for the callback
      -------------------------------------------------*/
      InterruptDetail detail;
      detail.channel = static_cast<Channel>( ch );
      detail.isr = Interrupt::EOC_SEQUENCE;

      /*-------------------------------------------------
      Empty the queue
      -------------------------------------------------*/
      while( nextSample( static_cast<Channel>( ch ), detail.data ) )
      {
        if( mCallbacks[ EnumValue( detail.isr ) ] )
        {
          mCallbacks[ EnumValue( detail.isr ) ]( detail );
        }
      }
    }

  }


  Chimera::Status_t Driver::open( const Chimera::ADC::DriverConfig &cfg )
  {
    /*-------------------------------------------------
    Validate inputs. Currently all basic configuration
    options are supported by this driver.
    -------------------------------------------------*/
    auto driver = LLD::getDriver( cfg.periph );
    if ( driver )
    {
      mConfig = cfg;
      mPeriph = cfg.periph;
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

#if defined( STM32L432xx )
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

#elif defined( STM32F446xx )
    if ( cycles < 4 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_3 );
    }
    else if ( cycles < 16 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_15 );
    }
    else if ( cycles < 29 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_28 );
    }
    else if ( cycles < 57 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_56 );
    }
    else if ( cycles < 85 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_84 );
    }
    else if ( cycles < 113 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_112 );
    }
    else if ( cycles < 145 )
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_144 );
    }
    else
    {
      return driver->setSampleTime( ch, LLD::SampleTime::SMP_480 );
    }
#endif
  }


  Chimera::ADC::Sample Driver::sampleChannel( const Chimera::ADC::Channel ch )
  {
    if ( !( ch < Chimera::ADC::Channel::NUM_OPTIONS ) )
    {
      return Chimera::ADC::Sample();
    }

    return LLD::getDriver( mConfig.periph )->sampleChannel( ch );
  }


  Chimera::Status_t Driver::configSequence( const Chimera::ADC::SequenceInit &cfg )
  {
    return LLD::getDriver( mConfig.periph )->setupSequence( cfg );
  }


  void Driver::startSequence()
  {
    LLD::getDriver( mConfig.periph )->startSequence();
  }


  void Driver::stopSequence()
  {
    LLD::getDriver( mConfig.periph )->stopSequence();
  }


  bool Driver::nextSample( const Chimera::ADC::Channel ch, Chimera::ADC::Sample &sample )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------
    Select the queue associated with this instance
    -------------------------------------------------*/
    Thor::LLD::ADC::PeriphQueue *queue = nullptr;
    switch( mPeriph )
    {
      case Peripheral::ADC_0:
        queue = &Thor::LLD::ADC::ADC1_Queue;
        break;

      default:
        return false;
        break;
    }

    /*-------------------------------------------------
    Return the next data in the queue if it exists
    -------------------------------------------------*/
    bool has_data = (*queue)[ EnumValue( ch ) ]->pop( sample );
    return has_data;
  }


  void Driver::onInterrupt( const Chimera::ADC::Interrupt bmSignal, Chimera::ADC::ISRCallback cb )
  {
    using namespace Chimera::ADC;

    if( bmSignal < Interrupt::NUM_OPTIONS )
    {
      mCallbacks[ EnumValue( bmSignal ) ] = cb;
    }
  }


  float Driver::toVoltage( const Chimera::ADC::Sample sample )
  {
    return LLD::getDriver( mConfig.periph )->toVoltage( sample );
  }

}    // namespace Thor::ADC

#endif /* THOR_HLD_ADC */
