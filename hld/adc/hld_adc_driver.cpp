/********************************************************************************
 *  File Name:
 *    hld_adc_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor ADC interface.
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Aurora/logging>
#include <Aurora/utility>
#include <Chimera/adc>
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/utility>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/interrupt>
#include <bitset>
#include <cstring>

#if defined( THOR_ADC )
namespace Chimera::ADC
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::ADC;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_ADC_PERIPHS;
  static constexpr size_t NUM_ISR_SIG = LLD::NUM_ADC_IRQ_HANDLERS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ThorImpl
  {
    LLD::Driver_rPtr           lldriver;
    Chimera::ADC::DriverConfig mConfig;  /**< Driver configuration settings */
    Chimera::ADC::SamplingMode mSeqMode; /**< Sequence sampling mode */

    /**
     *  User-space handler for processing ISR events
     *  @return void
     */
    void postISRProcessing()
    {
    }
  };

  /*---------------------------------------------------------------------------
  Variables
  ---------------------------------------------------------------------------*/
  static size_t               s_driver_initialized;        /**< Tracks the module level initialization state */
  static Chimera::ADC::Driver s_raw_driver[ NUM_DRIVERS ]; /**< Driver objects */
  static ThorImpl             s_impl_driver[ NUM_DRIVERS ];


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
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
      /*-----------------------------------------------------------------------
      Wait for something to wake this thread
      -----------------------------------------------------------------------*/
      if ( !this_thread::receiveTaskMsg( msg, TIMEOUT_BLOCK ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Handle every ISR. Don't know which triggered this
      -----------------------------------------------------------------------*/
      if ( msg & ITCMsg::TSK_MSG_ISR_DATA_READY )
      {
        for ( size_t index = 0; index < ARRAY_COUNT( s_impl_driver ); index++ )
        {
          s_impl_driver[ index ].postISRProcessing();
        }
      }

      // TODO: Handle the error case
    }
  }


  static Chimera::Status_t impl_initialize()
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
    Task       userThread;
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


  static Chimera::Status_t impl_reset()
  {
    return Chimera::Status::OK;
  }


  static Driver_rPtr impl_getDriver( const Chimera::ADC::Peripheral periph )
  {
    if ( auto idx = LLD::getResourceIndex( periph ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return &s_raw_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::ADC::DriverConfig &cfg )
  {
    /*-------------------------------------------------------------------------
    Ensure the peripheral is valid
    -------------------------------------------------------------------------*/
    auto idx = LLD::getResourceIndex( cfg.periph );
    if ( idx == ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Configure local implementation details
    -------------------------------------------------------------------------*/
    s_impl_driver[ idx ].lldriver = LLD::getDriver( cfg.periph );
    s_impl_driver[ idx ].mConfig  = cfg;

    mImpl = &s_impl_driver[ idx ];

    /*-------------------------------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------------------------------*/
    return s_impl_driver[ idx ].lldriver->initialize( cfg );
  }


  void Driver::close()
  {
    /*-------------------------------------------------------------------------
    Reset hardware resources
    -------------------------------------------------------------------------*/
    auto lld = reinterpret_cast<ThorImpl *>( mImpl )->lldriver;
    lld->reset();
    lld->clockReset();
    lld->clockDisable();
  }


  Chimera::Status_t Driver::setSampleTime( const Chimera::ADC::Channel ch, const size_t cycles )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !( ch < Chimera::ADC::Channel::NUM_OPTIONS ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Convert the sample times
    -------------------------------------------------------------------------*/
    auto lld = reinterpret_cast<ThorImpl *>( mImpl )->lldriver;

#if defined( STM32L432xx )
    if ( cycles < 3 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_2P5 );
    }
    else if ( cycles < 7 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_6P5 );
    }
    else if ( cycles < 13 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_12P5 );
    }
    else if ( cycles < 25 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_24P5 );
    }
    else if ( cycles < 48 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_47P5 );
    }
    else if ( cycles < 93 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_92P5 );
    }
    else if ( cycles < 248 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_247P5 );
    }
    else
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_640P5 );
    }

#elif defined( STM32F446xx )
    if ( cycles < 4 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_3 );
    }
    else if ( cycles < 16 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_15 );
    }
    else if ( cycles < 29 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_28 );
    }
    else if ( cycles < 57 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_56 );
    }
    else if ( cycles < 85 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_84 );
    }
    else if ( cycles < 113 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_112 );
    }
    else if ( cycles < 145 )
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_144 );
    }
    else
    {
      return lld->setSampleTime( ch, LLD::SampleTime::SMP_480 );
    }
#endif
  }


  Chimera::ADC::Sample Driver::sampleChannel( const Chimera::ADC::Channel ch )
  {
    if ( !( ch < Chimera::ADC::Channel::NUM_OPTIONS ) )
    {
      return Chimera::ADC::Sample();
    }

    return reinterpret_cast<ThorImpl *>( mImpl )->lldriver->sampleChannel( ch );
  }


  Chimera::Status_t Driver::configSequence( const Chimera::ADC::SequenceInit &cfg )
  {
    return reinterpret_cast<ThorImpl *>( mImpl )->lldriver->setupSequence( cfg );
  }


  void Driver::startSequence()
  {
    reinterpret_cast<ThorImpl *>( mImpl )->lldriver->startSequence();
  }


  void Driver::stopSequence()
  {
    reinterpret_cast<ThorImpl *>( mImpl )->lldriver->stopSequence();
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
    if ( !ch_arr || !sample_arr || !size )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Synchronize the HLD queue with the DMA buffers
    -------------------------------------------------------------------------*/
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    if ( ( impl->mSeqMode == SamplingMode::CONTINUOUS ) || ( impl->mSeqMode == SamplingMode::TRIGGER ) )
    {
      impl->lldriver->stopSequence();
      impl->lldriver->syncSequence();
      impl->lldriver->startSequence();
    }
    else /* One-shot software triggered. Should be safe to pull directly. */
    {
      impl->lldriver->syncSequence();
    }

    /*-------------------------------------------------------------------------
    Select the queue associated with this instance
    -------------------------------------------------------------------------*/
    Thor::LLD::ADC::PeriphQueue *queue = nullptr;
    switch ( impl->mConfig.periph )
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

    for ( size_t idx = 0; idx < size; idx++ )
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
    reinterpret_cast<ThorImpl *>( mImpl )->lldriver->onInterrupt( signal, cb );
  }


  float Driver::toVoltage( const Chimera::ADC::Sample &sample )
  {
    return reinterpret_cast<ThorImpl *>( mImpl )->lldriver->toVoltage( sample );
  }

}    // namespace Chimera::ADC


namespace Chimera::ADC::Backend
{
  Chimera::Status_t registerDriver( Chimera::ADC::Backend::DriverConfig &registry )
  {
    registry.isSupported      = true;
    registry.getDriver        = ::Chimera::ADC::impl_getDriver;
    registry.initialize       = ::Chimera::ADC::impl_initialize;
    registry.reset            = ::Chimera::ADC::impl_reset;
    registry.featureSupported = ::Thor::LLD::ADC::featureSupported;
    return Chimera::Status::OK;
  }
}    // namespace Chimera::ADC::Backend

#endif /* THOR_ADC */
