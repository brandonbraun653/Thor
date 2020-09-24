/********************************************************************************
 *  File Name:
 *    hld_spi_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera SPI driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/spi>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/spi/spi_intf.hpp>
#include <Thor/lld/interface/spi/spi_detail.hpp>
#include <Thor/lld/interface/spi/spi_prv_data.hpp>

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::SPI;
namespace LLD = ::Thor::LLD::SPI;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = ::LLD::NUM_SPI_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static Chimera::SPI::Driver s_raw_driver[ NUM_DRIVERS ];
static Chimera::SPI::Driver_sPtr s_shared_driver[ NUM_DRIVERS ];


namespace Chimera::SPI::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    for ( size_t x = 0; x < NUM_DRIVERS; x++ )
    {
      s_shared_driver[ x ] = Chimera::SPI::Driver_sPtr( &s_raw_driver[ x ] );
    }

    return Thor::SPI::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::SPI::reset();
  }


  Driver_sPtr getDriver( const Channel channel )
  {
    auto idx = ::LLD::getResourceIndex( channel );
    return s_shared_driver[ idx ];
  }


  Chimera::Status_t registerDriver( Chimera::SPI::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_SPI )
    registry.isSupported = true;
    registry.getDriver   = getDriver;
    registry.initialize  = initialize;
    registry.reset       = reset;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::SPI::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_HLD_SPI */
  }
}    // namespace Chimera::SPI::Backend


namespace Chimera::SPI
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
  Chimera::Status_t Driver::init( const Chimera::SPI::DriverConfig &setupStruct )
  {
    mDriver = ::HLD::getDriver( setupStruct.HWInit.hwChannel );
    return static_cast<::HLD::Driver_rPtr>( mDriver )->init( setupStruct );
  }


  Chimera::SPI::DriverConfig Driver::getInit()
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getInit();
  }


  Chimera::Status_t Driver::deInit()
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->deInit();
  }


  Chimera::Status_t Driver::setChipSelect( const Chimera::GPIO::State value )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setChipSelect( value );
  }


  Chimera::Status_t Driver::setChipSelectControlMode( const Chimera::SPI::CSMode mode )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setChipSelectControlMode( mode );
  }


  Chimera::Status_t Driver::writeBytes( const void *const txBuffer, const size_t length )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->writeBytes( txBuffer, length );
  }


  Chimera::Status_t Driver::readBytes( void *const rxBuffer, const size_t length )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->readBytes( rxBuffer, length );
  }


  Chimera::Status_t Driver::readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->readWriteBytes( txBuffer, rxBuffer, length );
  }


  Chimera::Status_t Driver::setPeripheralMode( const Chimera::Hardware::PeripheralMode mode )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setPeripheralMode( mode );
  }


  Chimera::Status_t Driver::setClockFrequency( const size_t freq, const size_t tolerance )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->setClockFrequency( freq, tolerance );
  }


  size_t Driver::getClockFrequency()
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->getClockFrequency();
  }


  /*-------------------------------------------------
  Interface: Listener
  -------------------------------------------------*/
  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->registerListener( listener, timeout, registrationID );
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->removeListener( registrationID, timeout );
  }


  /*-------------------------------------------------
  Interface: AsyncIO
  -------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, timeout );
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                           const size_t timeout )
  {
    return static_cast<::HLD::Driver_rPtr>( mDriver )->await( event, notifier, timeout );
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

}
