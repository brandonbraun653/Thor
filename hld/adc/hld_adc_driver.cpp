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

      // TODO: Handle the error case
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
  Driver::Driver() :
      mPeriph( Chimera::ADC::Peripheral::UNKNOWN ), mConfig( {} ), mSeqMode( Chimera::ADC::SamplingMode::UNKNOWN )
  {
    mConfig.clear();
  }


  Driver::~Driver()
  {
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


  bool Driver::nextSeqSample( const Chimera::ADC::Channel ch, Chimera::ADC::Sample &sample )
  {
    static constexpr size_t SAMPLE_SIZE = 1;

    return SAMPLE_SIZE == multiSeqSample( &ch, &sample, SAMPLE_SIZE );
  }


  size_t Driver::multiSeqSample( const Chimera::ADC::Channel *ch_arr, Chimera::ADC::Sample *sample_arr, const size_t size )
  {
    using namespace Chimera::ADC;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !ch_arr || !sample_arr || !size )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Synchronize the HLD queue with the DMA buffers
    -------------------------------------------------------------------------*/
    auto driver = LLD::getDriver( mConfig.periph );
    if( ( mSeqMode == SamplingMode::CONTINUOUS ) || ( mSeqMode == SamplingMode::TRIGGER ) )
    {
      driver->stopSequence();
      driver->syncSequence();
      driver->startSequence();
    }
    else /* One-shot software triggered. Should be safe to pull directly. */
    {
      driver->syncSequence();
    }

    /*-------------------------------------------------------------------------
    Select the queue associated with this instance
    -------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------
    Return as much data as available
    -------------------------------------------------------------------------*/
    size_t num_good_retrievals = 0;

    for( size_t idx = 0; idx < size; idx++ )
    { /* clang-format off */
      const size_t req_channel = static_cast<size_t>( ch_arr[ idx ] );

      if ( ( req_channel >= queue->size() )
        || ( queue->at( req_channel )->pop( sample_arr[ idx ] ) == false ) )
      {
        sample_arr[ idx ].clear();
      }
      else
      {
        num_good_retrievals++;
      }
    } /* clang-format on */

    return num_good_retrievals;
  }

  void Driver::onInterrupt( const Chimera::ADC::Interrupt signal, Chimera::ADC::ISRCallback cb )
  {
    LLD::getDriver( mConfig.periph )->onInterrupt( signal, cb );
  }


  float Driver::toVoltage( const Chimera::ADC::Sample sample )
  {
    return LLD::getDriver( mConfig.periph )->toVoltage( sample );
  }


  void Driver::postISRProcessing()
  {
    using namespace Chimera::ADC;


    /*-------------------------------------------------------------------------
    Need to figure out the architecture for how I want to move ADC data around.
    Currently this assumes notification to the user via callbacks when a single
    transfer is complete (one-shot mode). The downside is that any user calls to
    nextSeqSample will get choked because this high priority thread is emptying the
    queue.

    In continuous mode, this never runs and it's up to the user to call
    nextSeqSample to empty the queue. This makes some sense.

    Perhaps add a guard to only allow this to execute in one-shot mode? Where
    should this behavior be documented? Chimera?
    -------------------------------------------------------------------------*/

    // for( size_t ch = 0; ch < Thor::LLD::ADC::NUM_ADC_CHANNELS_PER_PERIPH; ch++ )
    // {
    //   /*-----------------------------------------------------------------------
    //   Populate the interrupt information for the callback
    //   -----------------------------------------------------------------------*/
    //   InterruptDetail detail;
    //   detail.channel = static_cast<Channel>( ch );
    //   detail.isr = Interrupt::EOC_SEQUENCE;

    //   /*-----------------------------------------------------------------------
    //   Empty the queue
    //   -----------------------------------------------------------------------*/
    //   while( nextSeqSample( static_cast<Channel>( ch ), detail.data ) )
    //   {
    //     if( mCallbacks[ EnumValue( detail.isr ) ] )
    //     {
    //       mCallbacks[ EnumValue( detail.isr ) ]( detail );
    //     }
    //   }
    // }
  }

}    // namespace Thor::ADC

#endif /* THOR_HLD_ADC */
