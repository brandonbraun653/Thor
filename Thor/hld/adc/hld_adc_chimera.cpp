/********************************************************************************
 *  File Name:
 *    hld_adc_chimera.cpp
 *
 *  Description:
 *    Implementation of Chimera ADC driver hooks
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/adc>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/adc>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/adc/adc_intf.hpp>
#include <Thor/lld/interface/adc/adc_detail.hpp>
#include <Thor/lld/interface/adc/adc_prv_data.hpp>

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::ADC;
namespace LLD = ::Thor::LLD::ADC;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_ADC_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
#if defined( THOR_HLD_ADC )
static Chimera::ADC::Driver s_raw_driver[ NUM_DRIVERS ];
#endif  /* THOR_HLD_ADC */

namespace Chimera::ADC::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
#if defined( THOR_HLD_ADC )
  Chimera::Status_t initialize()
  {
    return Thor::ADC::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::ADC::reset();
  }


  Chimera::ADC::Driver_sPtr getDriver( const Converter periph )
  {
    auto idx = ::LLD::getResourceIndex( periph );
    if( idx < NUM_DRIVERS )
    {
      return Chimera::ADC::Driver_sPtr( &s_raw_driver[ idx ] );
    }
    else
    {
      return nullptr;
    }
  }


  bool featureSupported( const Converter periph, const Feature feature )
  {
    return ::LLD::featureSupported( periph, feature );
  }

#endif  /* THOR_HLD_ADC */

  Chimera::Status_t registerDriver( Chimera::ADC::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_ADC )
    registry.isSupported      = true;
    registry.getDriver        = getDriver;
    registry.initialize       = initialize;
    registry.reset            = reset;
    registry.featureSupported = featureSupported;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::ADC::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif    // THOR_HLD_ADC
  }
}    // namespace Chimera::ADC::Backend


namespace Chimera::ADC
{
  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mDriver( nullptr )
  {
  }


  Driver::~Driver()
  {
  }


  /*-------------------------------------------------
  Interface: Hardware
  -------------------------------------------------*/
  Chimera::Status_t Driver::open( const DriverConfig &cfg )
  {
    mDriver = reinterpret_cast<void *>( ::HLD::getDriver( cfg.periph ) );

    if ( mDriver )
    {
      return static_cast<::HLD::Driver_rPtr>( mDriver )->open( cfg );
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
  }


  void Driver::close()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->close();
  }


  void Driver::setPowerState( const bool state )
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->setPowerState( state );
  }


  Chimera::ADC::Sample_t Driver::sampleChannel( const Chimera::ADC::Channel ch )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->sampleChannel( ch );
  }


  Chimera::ADC::Sample_t Driver::sampleSensor( const Chimera::ADC::Sensor sensor )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->sampleSensor( sensor );
  }


  Chimera::Status_t Driver::groupConfig( const Chimera::ADC::GroupInit &cfg )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->groupConfig( cfg );
  }


  Chimera::Status_t Driver::groupStartSample( const Chimera::ADC::SampleGroup grp )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->groupStartSample( grp );
  }


  Chimera::Status_t Driver::groupGetSample( const Chimera::ADC::SampleGroup grp, Chimera::ADC::Sample_t *const out,
                                            const size_t len )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->groupGetSample( grp, out, len );
  }


  Chimera::Status_t Driver::groupSetDMABuffer( const Chimera::ADC::SampleGroup grp, Chimera::ADC::Sample_t *const out,
                                               const size_t len )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->groupSetDMABuffer( grp, out, len );
  }


  Chimera::Status_t Driver::setSampleTime( const Chimera::ADC::Channel ch, const size_t cycles )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setSampleTime( ch, cycles );
  }


  void Driver::setWatchdogThreshold( const Chimera::ADC::Watchdog wd, const Chimera::ADC::Sample_t low,
                                     const Chimera::ADC::Sample_t high )
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->setWatchdogThreshold( wd, low, high );
  }


  void Driver::onInterrupt( const Chimera::ADC::Interrupt bmSignal, Chimera::ADC::ISRCallback cb )
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->onInterrupt( bmSignal, cb );
  }


  float Driver::sampleToVoltage( const Chimera::ADC::Sample_t sample )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->sampleToVoltage( sample );
  }


  float Driver::sampleToJunctionTemperature( const Sample_t sample )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->sampleToJunctionTemperature( sample );
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void Driver::lock()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->lock();
  }


  void Driver::lockFromISR()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->lockFromISR();
  }


  bool Driver::try_lock_for( const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->try_lock_for( timeout );
  }


  void Driver::unlock()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlock();
  }


  void Driver::unlockFromISR()
  {
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlockFromISR();
  }
}    // namespace Chimera::ADC
