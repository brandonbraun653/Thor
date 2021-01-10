/********************************************************************************
 *  File Name:
 *    hld_adc_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor ADC interface.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
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
#include <Thor/lld/interface/adc/adc_intf.hpp>
#include <Thor/lld/interface/adc/adc_detail.hpp>
#include <Thor/lld/interface/adc/adc_prv_data.hpp>

#if defined( THOR_HLD_ADC )

namespace Thor::ADC
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  namespace HLD = ::Thor::ADC;
  namespace LLD = ::Thor::LLD::ADC;

  using ThreadHandle = Chimera::Threading::detail::native_thread_handle_type;
  using BinarySemphr = Chimera::Threading::BinarySemaphore;
  using ThreadFunctn = Chimera::Function::void_func_void_ptr;

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
  static HLD::Driver_rPtr hld_shared[ NUM_DRIVERS ];         /**< Shared references to driver objects */
  static ThreadHandle s_user_isr_handle[ NUM_DRIVERS ];      /**< Handle to the ISR post processing thread */
  static ThreadFunctn s_user_isr_thread_func[ NUM_DRIVERS ]; /**< RTOS aware function to execute at end of ISR */

  /**
   *  Cache for holding the last set of samples performed on each channel
   */
  static Chimera::ADC::Sample_t last_samples[ NUM_DRIVERS ][ LLD::NUM_ADC_CHANNELS_PER_PERIPH ];

  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
  static void ADC1ISRPostProcessorThread( void *argument )
  {
    using namespace Chimera::Threading;

    constexpr auto index = LLD::ADC1_RESOURCE_INDEX;
    ThreadMsg tskMsg     = TSK_MSG_NOP;

    while ( 1 )
    {
      auto rcvd = this_thread::receiveTaskMsg( tskMsg, TIMEOUT_BLOCK );
      if ( rcvd && ( tskMsg == TSK_MSG_ISR_HANDLER ) )
      {
        hld_driver[ index ].postISRProcessing();
      }
    }
  }
#endif

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
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
    Initialize ISR post-processing routines
    ------------------------------------------------*/
#if defined( STM32_ADC1_PERIPH_AVAILABLE )
    s_user_isr_thread_func[ LLD::ADC1_RESOURCE_INDEX ] = ADC1ISRPostProcessorThread;
#endif

    /*------------------------------------------------
    Initialize local memory
    ------------------------------------------------*/
    // Driver pointers
    for ( size_t x = 0; x < NUM_DRIVERS; x++ )
    {
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_ADC )
      hld_shared[ x ] = HLD::Driver_rPtr( new HLD::Driver() );
#else
      hld_shared[ x ] = HLD::Driver_rPtr( &hld_driver[ x ] );
#endif
    }

    // Sample Cache
    auto const arr_len = NUM_DRIVERS * LLD::NUM_ADC_CHANNELS_PER_PERIPH * sizeof( Chimera::ADC::Sample_t );
    memset( last_samples, 0, arr_len );


    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }


  Chimera::Status_t reset()
  {
    /*------------------------------------------------
    Only allow clearing of local data during testing
    ------------------------------------------------*/
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_ADC )
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    for ( auto x = 0; x < NUM_DRIVERS; x++ )
    {
      hld_shared[ x ].reset();
    }
#endif

    return Chimera::Status::OK;
  }


  Driver_rPtr getDriver( const Chimera::ADC::Converter periph )
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
  Driver::Driver() : mPeriph( Chimera::ADC::Converter::UNKNOWN )
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
    Reset hld resources
    -------------------------------------------------*/
    memset( last_samples[ lldResourceIndex ], 0, LLD::NUM_ADC_CHANNELS_PER_PERIPH );

    /*------------------------------------------------
    Register the ISR post processor threads
    ------------------------------------------------*/
    if ( s_user_isr_thread_func[ lldResourceIndex ] )
    {
      std::array<char, 10> tmp;
      tmp.fill( 0 );
      snprintf( tmp.data(), tmp.size(), "PP_ADC%d", lldResourceIndex );
      std::string_view threadName = tmp.data();

      Chimera::Threading::Thread thread;
      thread.initialize( s_user_isr_thread_func[ lldResourceIndex ], nullptr, Chimera::Threading::Priority::LEVEL_5,
                         STACK_BYTES( 250 ), threadName );

      LLD::ISRThreadId[ lldResourceIndex ]  = thread.start();
      s_user_isr_handle[ lldResourceIndex ] = thread.native_handle();
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


  void Driver::setPowerState( const bool state )
  {
  }


  Chimera::ADC::Sample_t Driver::sampleChannel( const Chimera::ADC::Channel ch )
  {
    if ( !( ch < Chimera::ADC::Channel::NUM_OPTIONS ) )
    {
      return Chimera::ADC::INVALID_SAMPLE;
    }

    return LLD::getDriver( mConfig.periph )->sampleChannel( ch );
  }


  Chimera::ADC::Sample_t Driver::sampleSensor( const Chimera::ADC::Sensor sensor )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if ( !( sensor < Chimera::ADC::Sensor::NUM_OPTIONS ) )
    {
      return Chimera::ADC::INVALID_SAMPLE;
    }

    /*-------------------------------------------------
    Map the sensor into the project's ADC channels and
    then perform the conversion.
    -------------------------------------------------*/
    auto channel = LLD::ConfigMap::SensorToChannel[ static_cast<size_t>( sensor ) ];
    return LLD::getDriver( mConfig.periph )->sampleChannel( channel );
  }


  Chimera::Status_t Driver::groupConfig( const Chimera::ADC::GroupInit &cfg )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::groupStartSample( const Chimera::ADC::SampleGroup grp )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::groupGetSample( const Chimera::ADC::SampleGroup grp, Chimera::ADC::Sample_t *const out,
                                            const size_t len )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/


    auto lld = LLD::getDriver( mConfig.periph );

    /*-------------------------------------------------
    Safely copy out the current data
    -------------------------------------------------*/
    Chimera::ADC::Sample_t tmp[ LLD::NUM_ADC_CHANNELS_PER_PERIPH ];

    lld->enterCriticalSection();
    memcpy( tmp, last_samples[ 0 ], ARRAY_BYTES( tmp ) );
    lld->exitCriticalSection();

    /*-------------------------------------------------
    Store the data into the user's buffer
    -------------------------------------------------*/

    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::groupSetDMABuffer( const Chimera::ADC::SampleGroup grp, Chimera::ADC::Sample_t *const out,
                                               const size_t len )
  {
    return Chimera::Status::NOT_SUPPORTED;
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


  void Driver::setWatchdogThreshold( const Chimera::ADC::Watchdog wd, const Chimera::ADC::Sample_t low,
                                     const Chimera::ADC::Sample_t high )
  {
  }


  void Driver::onInterrupt( const Chimera::ADC::Interrupt bmSignal, Chimera::ADC::ISRCallback cb )
  {
  }


  float Driver::sampleToVoltage( const Chimera::ADC::Sample_t sample )
  {
    return LLD::getDriver( mConfig.periph )->sampleToVoltage( sample );
  }


  float Driver::sampleToJunctionTemperature( const Chimera::ADC::Sample_t sample )
  {
    return LLD::getDriver( mConfig.periph )->sampleToTemp( sample );
  }
}    // namespace Thor::ADC

#endif /* THOR_HLD_ADC */
