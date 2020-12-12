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
#include <Chimera/common>
#include <Chimera/adc>
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/adc>
#include <Thor/lld/interface/adc/adc_intf.hpp>
#include <Thor/lld/interface/adc/adc_detail.hpp>

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

  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static size_t s_driver_initialized;                /**< Tracks the module level initialization state */
  static HLD::Driver hld_driver[ NUM_DRIVERS ];      /**< Driver objects */
  static HLD::Driver_sPtr hld_shared[ NUM_DRIVERS ]; /**< Shared references to driver objects */

  /**
   *  Cache for holding the last set of samples performed on each channel
   */
  static Chimera::ADC::Sample_t last_samples[ NUM_DRIVERS ][ LLD::NUM_ADC_CHANNELS_PER_PERIPH ];

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
    Initialize local memory
    ------------------------------------------------*/
    // Driver pointers
    for ( size_t x = 0; x < NUM_DRIVERS; x++ )
    {
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_ADC )
      hld_shared[ x ] = HLD::Driver_sPtr( new HLD::Driver() );
#else
      hld_shared[ x ] = HLD::Driver_sPtr( &hld_driver[ x ] );
#endif
    }

    // Sample Cache
    auto const arr_len = NUM_DRIVERS * LLD::NUM_ADC_CHANNELS_PER_PERIPH * sizeof( Chimera::ADC::Sample_t );
    memset( last_samples, 0, arr_len );

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    result = Thor::LLD::ADC::initialize();

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


  Driver_sPtr getDriverShared( const Chimera::ADC::Converter periph )
  {
    if ( auto idx = LLD::getResourceIndex( periph ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return hld_shared[ idx ];
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


  Chimera::Status_t Driver::open( const Chimera::ADC::DriverConfig &cfg )
  {
    /*-------------------------------------------------
    Validate inputs. Currently all basic configuration
    options are supported by this driver.
    -------------------------------------------------*/
    auto lld = LLD::getDriver( mConfig.periph );
    auto idx = LLD::getResourceIndex( mConfig.periph );

    if ( lld )
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
    memset( last_samples[ idx ], 0, LLD::NUM_ADC_CHANNELS_PER_PERIPH );

    /*-------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------*/
    return lld->initialize( mConfig );
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
  }


  Chimera::ADC::Sample_t Driver::sampleSensor( const Chimera::ADC::Sensor sensor )
  {
  }


  Chimera::Status_t Driver::groupConfig( const Chimera::ADC::GroupInit &cfg )
  {
  }


  Chimera::Status_t Driver::groupStartSample( const Chimera::ADC::SampleGroup grp )
  {
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
  }


  Chimera::Status_t Driver::groupSetDMABuffer( const Chimera::ADC::SampleGroup grp, Chimera::ADC::Sample_t *const out,
                                               const size_t len )
  {
  }


  Chimera::Status_t Driver::setSampleTime( const Chimera::ADC::Channel ch, const size_t cycles )
  {
  }


  void Driver::setWatchdogThreshold( const Chimera::ADC::Watchdog wd, const Chimera::ADC::Sample_t low,
                                     const Chimera::ADC::Sample_t high )
  {
  }


  void Driver::onInterrupt( const Chimera::ADC::Interrupt bmSignal, Chimera::ADC::ISRCallback cb )
  {
  }

}    // namespace Thor::ADC

#endif /* THOR_HLD_ADC */
