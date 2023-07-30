/******************************************************************************
 *  File Name:
 *    hld_sdio_driver.cpp
 *
 *  Description:
 *    SDIO driver for Thor
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/gpio>
#include <Chimera/peripheral>
#include <Chimera/sdio>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/sdio>

#if defined( THOR_SDIO )
namespace Chimera::SDIO
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::SDIO;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_SDIO_PERIPHS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  class ThorImpl
  {
  public:
    Chimera::Status_t powerOn();

    LLD::Driver_rPtr           lldriver;
    Chimera::SDIO::Driver_rPtr hldriver;
    CardInfo                   cardInfo;
    uint32_t                   clockSpeed;
    BusWidth                   busWidth;

  private:
    LLD::ErrorType acmd41Init();
  };

  /*---------------------------------------------------------------------------
  Variables
  ---------------------------------------------------------------------------*/
  static DeviceManager<Driver, Channel, NUM_DRIVERS>   s_raw_drivers;
  static DeviceManager<ThorImpl, Channel, NUM_DRIVERS> s_impl_drivers;

  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::SDIO::HWConfig &init )
  {
    /*-------------------------------------------------------------------------
    Ensure the AsyncIO driver is ready
    -------------------------------------------------------------------------*/
    this->initAIO();

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    auto idx = LLD::getResourceIndex( init.channel );
    if ( idx == ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Bind the driver to the correct LLD instance
    -------------------------------------------------------------------------*/
    auto impl = s_impl_drivers.getOrCreate( init.channel );
    mImpl     = reinterpret_cast<void *>( impl );
    RT_DBG_ASSERT( impl );

    impl->lldriver = LLD::getDriver( init.channel );
    impl->hldriver = this;
    RT_DBG_ASSERT( impl->lldriver );

    /*-------------------------------------------------------------------------
    Initialize the GPIO
    -------------------------------------------------------------------------*/
    Chimera::GPIO::Driver_rPtr gpio = nullptr;

    /* Clock Pin */
    gpio = Chimera::GPIO::getDriver( init.clkPin.port, init.clkPin.pin );
    RT_DBG_ASSERT( gpio );
    RT_DBG_ASSERT( gpio->init( init.clkPin ) == Chimera::Status::OK );

    /* Command Pin */
    gpio = Chimera::GPIO::getDriver( init.cmdPin.port, init.cmdPin.pin );
    RT_DBG_ASSERT( gpio );
    RT_DBG_ASSERT( gpio->init( init.cmdPin ) == Chimera::Status::OK );

    /* Data Pins */
    for ( size_t x = 0; x < ARRAY_COUNT( init.dxPin ); x++ )
    {
      gpio = Chimera::GPIO::getDriver( init.dxPin[ x ].port, init.dxPin[ x ].pin );
      RT_DBG_ASSERT( gpio );
      RT_DBG_ASSERT( gpio->init( init.dxPin[ x ] ) == Chimera::Status::OK );
    }

    /*-------------------------------------------------------------------------
    Initialize the HLD driver
    -------------------------------------------------------------------------*/
    memset( &impl->cardInfo, 0, sizeof( impl->cardInfo ) );
    impl->busWidth   = init.width;
    impl->clockSpeed = init.clockSpeed;

    /*-------------------------------------------------------------------------
    Initialize the LLD driver
    -------------------------------------------------------------------------*/
    return impl->lldriver->init();
  }


  Chimera::Status_t Driver::connect()
  {
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );
    RT_DBG_ASSERT( impl );
    RT_DBG_ASSERT( impl->lldriver );

    /*-------------------------------------------------------------------------
    Transition the HW to the power ON state
    -------------------------------------------------------------------------*/
    impl->lldriver->busClockDisable();
    if ( impl->lldriver->setPowerStateOn() == Chimera::Status::OK )
    {
      impl->lldriver->busClockEnable();
    }
    else
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Get the operating card voltage
    -------------------------------------------------------------------------*/
    if ( impl->powerOn() != Chimera::Status::OK )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Initialize the card
    -------------------------------------------------------------------------*/


    /*-------------------------------------------------------------------------
    Set the card block size
    -------------------------------------------------------------------------*/

    return Chimera::Status::FAIL;
  }


  void Driver::close()
  {
  }


  Chimera::Status_t Driver::write( const uint32_t address, const void *const buffer, const size_t length )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::read( const uint32_t address, void *const buffer, const size_t length )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::getCardStatus( CardStatus &status )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::getCardIdentity( CardIdentity &identity )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::getCardSpecificData( CardSpecificData &data )
  {
    return Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::getCardInfo( CardInfo &info )
  {
    return Chimera::Status::FAIL;
  }


  /*---------------------------------------------------------------------------
  Thor Driver Implementation
  ---------------------------------------------------------------------------*/
  Chimera::Status_t ThorImpl::powerOn()
  {
    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    LLD::ErrorType error = LLD::ERROR_NONE;

    /*-------------------------------------------------------------------------
    Send CMD0 to reset the card. Card is in the idle state after power up.
    -------------------------------------------------------------------------*/
    if ( error = lldriver->cmdGoIdleState(); error != LLD::ERROR_NONE )
    {
      goto exit_power_up;
    }

    /*-------------------------------------------------------------------------
    Figure out the card version by sending CMD8, which is only supported by
    high capacity cards like SDHC and SDXC. If the card does not support CMD8,
    it is a version 1.x card.
    -------------------------------------------------------------------------*/
    if ( error = lldriver->cmdOperCond(); error != LLD::ERROR_NONE )
    {
      cardInfo.version = CardVersion::CARD_V1_X;

      if ( error = lldriver->cmdGoIdleState(); error != LLD::ERROR_NONE )
      {
        goto exit_power_up;
      }
    }
    else
    {
      cardInfo.version = CardVersion::CARD_V2_X;
    }

    /*-------------------------------------------------------------------------
    Do the ACMD41 initialization sequence
    -------------------------------------------------------------------------*/
    error = acmd41Init();

    /*-------------------------------------------------------------------------
    Map into a simple pass/fail
    -------------------------------------------------------------------------*/
    exit_power_up:
      return ( error == LLD::ERROR_NONE ) ? Chimera::Status::OK : Chimera::Status::NOT_SUPPORTED;
  }

  /**
   * @brief Performs the ACMD41 initialization sequence
   * @see Physical Layer Simplified Specification Version 9.00 Section 4.2.3.1
   *
   * @return Card initialization status
   */
  LLD::ErrorType ThorImpl::acmd41Init()
  {
    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    LLD::ErrorType error        = LLD::ERROR_NONE;
    uint32_t       count        = 0;
    uint32_t       response     = 0;
    bool           device_ready = false;

    /*-------------------------------------------------------------------------
    Send CMD55 and CMD41 until the card indicates it is ready
    -------------------------------------------------------------------------*/
    while ( ( count < 0xFFFFu ) && !device_ready )
    {
      /*-----------------------------------------------------------------------
      Prepare for ACMD41 by sending CMD55, indicating the next command is an
      application specific command.
      -----------------------------------------------------------------------*/
      if ( error = lldriver->cmdAppCommand( 0u ); error != LLD::ERROR_NONE )
      {
        return error;
      }

      /*-----------------------------------------------------------------------
      Send the ACMD41 command with the broadest possible support. Don't bother
      with 1.8v signaling since the F4 drivers in Thor don't support it.
      -----------------------------------------------------------------------*/
      //constexpr uint32_t ACMD41_ARG = 0x40FF0000;
      constexpr uint32_t ACMD41_ARG = ACMD41_HIGH_CAPACITY_CARD | ACMD41_2V0_3V6_VOLT_WINDOW;

      if ( error = lldriver->cmdAppOperCommand( ACMD41_ARG ); error != LLD::ERROR_NONE )
      {
        return error;
      }

      response     = lldriver->cpsmGetResponse( LLD::ResponseMailbox::RESPONSE_1 );
      device_ready = ( response & ACMD41_INIT_COMPLETE ) == ACMD41_INIT_COMPLETE;
      count++;
    }

    /*-------------------------------------------------------------------------
    Save reported card capacity information
    -------------------------------------------------------------------------*/
    cardInfo.CardType = ( response & ACMD41_HIGH_CAPACITY_CARD ) ? CardType::CARD_SDHC_SDXC : CardType::CARD_SDSC;

    return error;
  }

}    // namespace Chimera::SDIO


namespace Chimera::SDIO::Backend
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static size_t s_driver_initialized;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initialize()
  {
    /*-------------------------------------------------------------------------
    Prevent multiple initializations
    -------------------------------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*-------------------------------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return LLD::initialize();
  }


  static Chimera::Status_t reset()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  static Driver_rPtr getDriver( const Chimera::SDIO::Channel channel )
  {
    if ( !LLD::isSupported( channel ) )
    {
      return nullptr;
    }

    return s_raw_drivers.getOrCreate( channel );
  }


  Chimera::Status_t registerDriver( Chimera::SDIO::Backend::DriverConfig &registry )
  {
    registry.isSupported = true;
    registry.getDriver   = ::Chimera::SDIO::Backend::getDriver;
    registry.initialize  = ::Chimera::SDIO::Backend::initialize;
    registry.reset       = ::Chimera::SDIO::Backend::reset;
    return Chimera::Status::OK;
  }
}    // namespace Chimera::SDIO::Backend
#endif /* THOR_SDIO */
