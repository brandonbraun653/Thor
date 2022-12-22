/******************************************************************************
 *  File Name:
 *    serial_driver.cpp
 *
 *  Description:
 *    Low level Serial driver interface layer
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/peripheral>
#include <Thor/lld/interface/inc/serial>
#include <Thor/lld/interface/inc/uart>
#include <Thor/lld/interface/inc/usart>

namespace Thor::LLD::Serial
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = Thor::LLD::Serial::NUM_SERIAL_PERIPHS;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static size_t                                                                s_driver_initialized;
  static Chimera::DeviceManager<Driver, Chimera::Serial::Channel, NUM_DRIVERS> s_raw_drivers;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------------------------------
    Prevent multiple initializations (need reset first)
    -------------------------------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

/*-------------------------------------------------------------------------
Initialize the low level drivers
-------------------------------------------------------------------------*/
#if defined( THOR_USART )
    RT_HARD_ASSERT( Chimera::Status::OK == Thor::LLD::USART::initialize() );
#endif
#if defined( THOR_UART )
    RT_HARD_ASSERT( Chimera::Status::OK == Thor::LLD::UART::initialize() );
#endif

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  void registerInterface( const Chimera::Serial::Channel channel, HwInterface *const intf )
  {
    /*-------------------------------------------------------------------------
    Input Protections
    -------------------------------------------------------------------------*/
    if ( ( channel >= Chimera::Serial::Channel::NUM_OPTIONS ) || ( intf == nullptr ) )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Assign the driver interface
    -------------------------------------------------------------------------*/
    auto driver     = s_raw_drivers.getOrCreate( channel );
    driver->mHWIntf = intf;
  }


  Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    return s_raw_drivers.get( channel );
  }


  bool isSupported( const Chimera::Serial::Channel channel )
  {
    return s_raw_drivers.get( channel ) != nullptr;
  }


  /*---------------------------------------------------------------------------
  Serial Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mHWIntf( 0 )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::Serial::Config &config )
  {
    RegConfig regCfg;
    regCfg.BaudRate = static_cast<uint32_t>( config.baud );

    if ( mHWIntf->periphType() == Chimera::Peripheral::Type::PERIPH_USART )
    {
      regCfg.Mode       = Thor::LLD::USART::Configuration::Modes::TX_RX;
      regCfg.Parity     = Thor::LLD::USART::ConfigMap::Parity[ EnumValue( config.parity ) ];
      regCfg.StopBits   = Thor::LLD::USART::ConfigMap::StopBits[ EnumValue( config.stopBits ) ];
      regCfg.WordLength = Thor::LLD::USART::ConfigMap::CharWidth[ EnumValue( config.width ) ];
    }

    return mHWIntf->init( regCfg );
  }


  Chimera::Status_t Driver::close()
  {
    return mHWIntf->deinit();
  }


  int Driver::write( const Chimera::Serial::TxfrMode mode, etl::span<uint8_t> &buffer )
  {
    return mHWIntf->transmit( mode, buffer );
  }


  int Driver::read( const Chimera::Serial::TxfrMode mode, etl::span<uint8_t> &buffer )
  {
    return mHWIntf->receive( mode, buffer );
  }


  Chimera::Status_t Driver::txStatus()
  {
    return mHWIntf->txTransferStatus();
  }


  Chimera::Status_t Driver::rxStatus()
  {
    return mHWIntf->rxTransferStatus();
  }


  Flag_t Driver::getFlags()
  {
    return mHWIntf->getFlags();
  }


  void Driver::clearFlags( const uint32_t flagBits )
  {
    mHWIntf->clearFlags( flagBits );
  }


  CDTCB *Driver::getTCB_TX()
  {
    return mHWIntf->getTCB_TX();
  }


  MDTCB *Driver::getTCB_RX()
  {
    return mHWIntf->getTCB_RX();
  }

}    // namespace Thor::LLD::Serial
