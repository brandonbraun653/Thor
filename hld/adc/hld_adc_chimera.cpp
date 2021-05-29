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
#include <Chimera/assert>
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
#if defined( THOR_HLD_ADC )
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_ADC_PERIPHS;
#endif  /* THOR_HLD_ADC */

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


  Chimera::ADC::Driver_rPtr getDriver( const Peripheral periph )
  {
    auto idx = ::LLD::getResourceIndex( periph );
    if( idx < NUM_DRIVERS )
    {
      return &s_raw_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  bool featureSupported( const Peripheral periph, const Feature feature )
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
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->close();
  }


  Chimera::Status_t Driver::setSampleTime( const Chimera::ADC::Channel ch, const size_t cycles )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setSampleTime( ch, cycles );
  }


  Chimera::ADC::Sample Driver::sampleChannel( const Chimera::ADC::Channel ch )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->sampleChannel( ch );
  }


  Chimera::Status_t Driver::configSequence( const Chimera::ADC::SequenceInit &cfg )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->configSequence( cfg );
  }


  void Driver::startSequence()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->startSequence();
  }


  void Driver::stopSequence()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->stopSequence();
  }


  bool Driver::nextSample( const Chimera::ADC::Channel ch, Chimera::ADC::Sample &sample )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->nextSample( ch, sample );
  }

  void Driver::onInterrupt( const Chimera::ADC::Interrupt bmSignal, Chimera::ADC::ISRCallback cb )
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->onInterrupt( bmSignal, cb );
  }


  float Driver::toVoltage( const Chimera::ADC::Sample sample )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->toVoltage( sample );
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void Driver::lock()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->lock();
  }


  void Driver::lockFromISR()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->lockFromISR();
  }


  bool Driver::try_lock_for( const size_t timeout )
  {
    RT_HARD_ASSERT( mDriver );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->try_lock_for( timeout );
  }


  void Driver::unlock()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlock();
  }


  void Driver::unlockFromISR()
  {
    RT_HARD_ASSERT( mDriver );
    static_cast<::HLD::Driver_rPtr>( mDriver )->unlockFromISR();
  }
}    // namespace Chimera::ADC
