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
    LLD::ErrorType    configureBusWidth( const Chimera::SDIO::BusWidth width );

    LLD::Driver_rPtr           lldriver;   /**< Low level driver instance */
    Chimera::SDIO::Driver_rPtr hldriver;   /**< High level driver instance */
    CardInfo                   cardInfo;   /**< Cached card information */
    uint32_t                   clockSpeed; /**< User configured data bus speed */
    BusWidth                   busWidth;   /**< User configured data bus width */
    uint16_t                   rca;        /**< Relative Card Address */
    uint32_t                   cid[ 4 ];   /**< Card reported CID register */
    uint32_t                   csd[ 4 ];   /**< Card reported CSD register */

  private:
    LLD::ErrorType acmd41Init();
    LLD::ErrorType busWidthEnable();
    LLD::ErrorType busWidthDisable();
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
    /*-------------------------------------------------------------------------
    Goes through the full initialization sequence for the SDIO card. See Figure
    4-2 and 4-13 in the Physical Layer Simplified Specification Version 9.00.
    -------------------------------------------------------------------------*/
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
    Move the card through the identification process
    -------------------------------------------------------------------------*/
    if ( impl->powerOn() != Chimera::Status::OK )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Request the CID from the card
    -------------------------------------------------------------------------*/
    if ( impl->lldriver->cmdSendCID() != LLD::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }

    impl->cid[ 0 ] = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_1 );
    impl->cid[ 1 ] = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_2 );
    impl->cid[ 2 ] = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_3 );
    impl->cid[ 3 ] = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_4 );

    CardIdentity tmpId;
    getCardIdentity( tmpId );

    /*-------------------------------------------------------------------------
    Request the relative address. Once complete, the card will transition from
    the identification state to the standby state.
    -------------------------------------------------------------------------*/
    if ( impl->lldriver->cmdSetRelAdd( &impl->rca ) != LLD::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Request the CSD from the card. This only works after the card has been
    placed into the standby state and is addressable.
    -------------------------------------------------------------------------*/
    if ( impl->lldriver->cmdSendCSD( impl->rca ) != LLD::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }

    impl->csd[ 0 ] = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_1 );
    impl->csd[ 1 ] = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_2 );
    impl->csd[ 2 ] = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_3 );
    impl->csd[ 3 ] = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_4 );

    CardSpecificData tmpCSD;
    getCardSpecificData( tmpCSD );

    /*-------------------------------------------------------------------------
    Select the card, transitioning to the transfer state.
    -------------------------------------------------------------------------*/
    if ( impl->lldriver->cmdSelDesel( impl->rca ) != LLD::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }

    /*-----------------------------------------------------------------------------
    Switch to the desired bus width
    -----------------------------------------------------------------------------*/
    if ( impl->configureBusWidth( impl->busWidth ) != LLD::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Go full send with the clock speed
    -------------------------------------------------------------------------*/
    if ( impl->lldriver->setBusFrequency( impl->clockSpeed ) != LLD::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }

    return Chimera::Status::OK;
  }


  void Driver::close()
  {
  }


  Chimera::Status_t Driver::writeBlock( const uint32_t blockAddress, const size_t blockCount, const void *const buffer,
                                        const size_t size )
  {
    // handle something about address division for SDXC cards?

    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto result = impl->lldriver->asyncWriteBlock( blockAddress, blockCount, buffer );

    return ( result == LLD::ERROR_NONE ) ? Chimera::Status::OK : Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::readBlock( const uint32_t blockAddress, const size_t blockCount, void *const buffer,
                                       const size_t size )
  {
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto result = impl->lldriver->asyncReadBlock( blockAddress, blockCount, buffer );

    return ( result == LLD::ERROR_NONE ) ? Chimera::Status::OK : Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::eraseBlock( const uint32_t blockAddress, const size_t blockCount )
  {
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    if ( auto error = impl->lldriver->cmdEraseStartAdd( blockAddress ); error != LLD::ErrorType::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }

    if ( auto error = impl->lldriver->cmdEraseEndAdd( blockAddress + ( 512 * blockCount ) );
         error != LLD::ErrorType::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }


    if ( auto error = impl->lldriver->cmdErase(); error != LLD::ErrorType::ERROR_NONE )
    {
      return Chimera::Status::FAIL;
    }

    return Chimera::Status::OK;
  }


  CardState Driver::getCardState()
  {
    auto           impl  = reinterpret_cast<ThorImpl *>( mImpl );
    CardState      state = CardState::CARD_ERROR;
    LLD::ErrorType error = LLD::ErrorType::ERROR_GENERAL_UNKNOWN_ERR;

    /*-------------------------------------------------------------------------
    Send CMD13 to get the card status
    -------------------------------------------------------------------------*/
    if ( error = impl->lldriver->cmdSendStatus( impl->rca ); error != LLD::ErrorType::ERROR_NONE )
    {
      return CardState::CARD_ERROR;
    }

    /*-------------------------------------------------------------------------
    Parse the response to get the card state
    -------------------------------------------------------------------------*/
    const uint32_t resp1 = impl->lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_1 );
    state                = static_cast<CardState>( ( resp1 >> 9U ) & 0x0FU );

    return state;
  }


  Chimera::Status_t Driver::getCardStatus( CardStatus &status )
  {
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    uint32_t       sd_status[ 16 ];
    LLD::ErrorType errorstate;

    errorstate = impl->lldriver->getSDStatus( impl->rca, &sd_status[ 0 ] );

    status.DataBusWidth = ( uint8_t )( ( sd_status[ 0 ] & 0xC0U ) >> 6U );
    status.SecuredMode  = ( uint8_t )( ( sd_status[ 0 ] & 0x20U ) >> 5U );
    status.CardType = ( uint16_t )( ( ( sd_status[ 0 ] & 0x00FF0000U ) >> 8U ) | ( ( sd_status[ 0 ] & 0xFF000000U ) >> 24U ) );

    status.ProtectedAreaSize  = ( ( ( sd_status[ 1 ] & 0xFFU ) << 24U ) | ( ( sd_status[ 1 ] & 0xFF00U ) << 8U ) |
                                 ( ( sd_status[ 1 ] & 0xFF0000U ) >> 8U ) | ( ( sd_status[ 1 ] & 0xFF000000U ) >> 24U ) );
    status.SpeedClass         = ( uint8_t )( sd_status[ 2 ] & 0xFFU );
    status.PerformanceMove    = ( uint8_t )( ( sd_status[ 2 ] & 0xFF00U ) >> 8U );
    status.AllocationUnitSize = ( uint8_t )( ( sd_status[ 2 ] & 0xF00000U ) >> 20U );
    status.EraseSize          = ( uint16_t )( ( ( sd_status[ 2 ] & 0xFF000000U ) >> 16U ) | ( sd_status[ 3 ] & 0xFFU ) );
    status.EraseTimeout       = ( uint8_t )( ( sd_status[ 3 ] & 0xFC00U ) >> 10U );
    status.EraseOffset        = ( uint8_t )( ( sd_status[ 3 ] & 0x0300U ) >> 8U );

    return ( errorstate == LLD::ErrorType::ERROR_NONE ) ? Chimera::Status::OK : Chimera::Status::FAIL;
  }


  Chimera::Status_t Driver::getCardIdentity( CardIdentity &identity )
  {
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    identity.ManufacturerID = ( uint8_t )( ( impl->cid[ 0 ] & 0xFF000000U ) >> 24U );
    identity.OEM_AppliID    = ( uint16_t )( ( impl->cid[ 0 ] & 0x00FFFF00U ) >> 8U );
    identity.ProdName1      = ( ( ( impl->cid[ 0 ] & 0x000000FFU ) << 24U ) | ( ( impl->cid[ 1 ] & 0xFFFFFF00U ) >> 8U ) );
    identity.ProdName2      = ( uint8_t )( impl->cid[ 1 ] & 0x000000FFU );
    identity.ProdRev        = ( uint8_t )( ( impl->cid[ 2 ] & 0xFF000000U ) >> 24U );
    identity.ProdSN         = ( ( ( impl->cid[ 2 ] & 0x00FFFFFFU ) << 8U ) | ( ( impl->cid[ 3 ] & 0xFF000000U ) >> 24U ) );
    identity.ManufactDate   = ( uint16_t )( ( impl->cid[ 3 ] & 0x000FFF00U ) >> 8U );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::getCardSpecificData( CardSpecificData &data )
  {
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    if ( ( impl->cardInfo.CardType != CARD_SDSC ) && ( impl->cardInfo.CardType != CARD_SDHC_SDXC ) )
    {
      return Chimera::Status::FAIL;
    }

    data.CSDStruct       = ( uint8_t )( ( impl->csd[ 0 ] & 0xC0000000U ) >> 30U );
    data.SysSpecVersion  = ( uint8_t )( ( impl->csd[ 0 ] & 0x3C000000U ) >> 26U );
    data.TAAC            = ( uint8_t )( ( impl->csd[ 0 ] & 0x00FF0000U ) >> 16U );
    data.NSAC            = ( uint8_t )( ( impl->csd[ 0 ] & 0x0000FF00U ) >> 8U );
    data.MaxBusClkFrec   = ( uint8_t )( impl->csd[ 0 ] & 0x000000FFU );
    data.CardComdClasses = ( uint16_t )( ( impl->csd[ 1 ] & 0xFFF00000U ) >> 20U );
    data.RdBlockLen      = ( uint8_t )( ( impl->csd[ 1 ] & 0x000F0000U ) >> 16U );
    data.PartBlockRead   = ( uint8_t )( ( impl->csd[ 1 ] & 0x00008000U ) >> 15U );
    data.WrBlockMisalign = ( uint8_t )( ( impl->csd[ 1 ] & 0x00004000U ) >> 14U );
    data.RdBlockMisalign = ( uint8_t )( ( impl->csd[ 1 ] & 0x00002000U ) >> 13U );
    data.DSRImpl         = ( uint8_t )( ( impl->csd[ 1 ] & 0x00001000U ) >> 12U );

    if ( impl->cardInfo.CardType == CARD_SDSC )
    {
      data.DeviceSize         = ( ( ( impl->csd[ 1 ] & 0x000003FFU ) << 2U ) | ( ( impl->csd[ 2 ] & 0xC0000000U ) >> 30U ) );
      data.MaxRdCurrentVDDMin = ( uint8_t )( ( impl->csd[ 2 ] & 0x38000000U ) >> 27U );
      data.MaxRdCurrentVDDMax = ( uint8_t )( ( impl->csd[ 2 ] & 0x07000000U ) >> 24U );
      data.MaxWrCurrentVDDMin = ( uint8_t )( ( impl->csd[ 2 ] & 0x00E00000U ) >> 21U );
      data.MaxWrCurrentVDDMax = ( uint8_t )( ( impl->csd[ 2 ] & 0x001C0000U ) >> 18U );
      data.DeviceSizeMul      = ( uint8_t )( ( impl->csd[ 2 ] & 0x00038000U ) >> 15U );

      impl->cardInfo.BlockNbr = ( data.DeviceSize + 1U );
      impl->cardInfo.BlockNbr *= ( 1UL << ( ( data.DeviceSizeMul & 0x07U ) + 2U ) );
      impl->cardInfo.BlockSize    = ( 1UL << ( data.RdBlockLen & 0x0FU ) );
      impl->cardInfo.LogBlockNbr  = ( impl->cardInfo.BlockNbr ) * ( ( impl->cardInfo.BlockSize ) / 512U );
      impl->cardInfo.LogBlockSize = 512U;
    }
    else if ( impl->cardInfo.CardType == CARD_SDHC_SDXC )
    {
      /* Byte 7 */
      data.DeviceSize         = ( ( ( impl->csd[ 1 ] & 0x0000003FU ) << 16U ) | ( ( impl->csd[ 2 ] & 0xFFFF0000U ) >> 16U ) );
      impl->cardInfo.BlockNbr = ( ( data.DeviceSize + 1U ) * 1024U );
      impl->cardInfo.LogBlockNbr  = impl->cardInfo.BlockNbr;
      impl->cardInfo.BlockSize    = 512U;
      impl->cardInfo.LogBlockSize = impl->cardInfo.BlockSize;
    }

    data.EraseGrSize         = ( uint8_t )( ( impl->csd[ 2 ] & 0x00004000U ) >> 14U );
    data.EraseGrMul          = ( uint8_t )( ( impl->csd[ 2 ] & 0x00003F80U ) >> 7U );
    data.WrProtectGrSize     = ( uint8_t )( impl->csd[ 2 ] & 0x0000007FU );
    data.WrProtectGrEnable   = ( uint8_t )( ( impl->csd[ 3 ] & 0x80000000U ) >> 31U );
    data.ManDeflECC          = ( uint8_t )( ( impl->csd[ 3 ] & 0x60000000U ) >> 29U );
    data.WrSpeedFact         = ( uint8_t )( ( impl->csd[ 3 ] & 0x1C000000U ) >> 26U );
    data.MaxWrBlockLen       = ( uint8_t )( ( impl->csd[ 3 ] & 0x03C00000U ) >> 22U );
    data.WriteBlockPaPartial = ( uint8_t )( ( impl->csd[ 3 ] & 0x00200000U ) >> 21U );
    data.ContentProtectAppli = ( uint8_t )( ( impl->csd[ 3 ] & 0x00010000U ) >> 16U );
    data.FileFormatGroup     = ( uint8_t )( ( impl->csd[ 3 ] & 0x00008000U ) >> 15U );
    data.CopyFlag            = ( uint8_t )( ( impl->csd[ 3 ] & 0x00004000U ) >> 14U );
    data.PermWrProtect       = ( uint8_t )( ( impl->csd[ 3 ] & 0x00002000U ) >> 13U );
    data.TempWrProtect       = ( uint8_t )( ( impl->csd[ 3 ] & 0x00001000U ) >> 12U );
    data.FileFormat          = ( uint8_t )( ( impl->csd[ 3 ] & 0x00000C00U ) >> 10U );
    data.ECC                 = ( uint8_t )( ( impl->csd[ 3 ] & 0x00000300U ) >> 8U );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::getCardInfo( CardInfo &info )
  {
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );
    info      = impl->cardInfo;
    return Chimera::Status::OK;
  }


  /*---------------------------------------------------------------------------
  Thor Driver Implementation
  ---------------------------------------------------------------------------*/
  /**
   * @brief Powers on the SDIO card and initializes it
   * @return Chimera::Status_t
   */
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
   * @brief Sets the bus width of the SDIO interface
   * @note Expected to be performed after the card is powered on and in a ready state
   *
   * @param width The bus width to use
   * @return LLD::ErrorType
   */
  LLD::ErrorType ThorImpl::configureBusWidth( const Chimera::SDIO::BusWidth width )
  {
    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    LLD::ErrorType error = LLD::ERROR_NONE;

    /*-------------------------------------------------------------------------
    Ignore locked cards
    -------------------------------------------------------------------------*/
    if ( this->cardInfo.CardType == CardType::CARD_SECURED )
    {
      return LLD::ErrorType::ERROR_UNSUPPORTED_FEATURE;
    }

    /*-------------------------------------------------------------------------
    Issue the command to change the bus width to the SD card
    -------------------------------------------------------------------------*/
    switch ( width )
    {
      case Chimera::SDIO::BusWidth::BUS_WIDTH_1BIT:
        error = this->busWidthDisable();
        break;

      case Chimera::SDIO::BusWidth::BUS_WIDTH_4BIT:
        error = this->busWidthEnable();
        break;

      default:
        error = LLD::ErrorType::ERROR_INVALID_PARAMETER;
        break;
    }

    return error;
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
      constexpr uint32_t ACMD41_ARG = ACMD41_HIGH_CAPACITY_CARD | ACMD41_2V0_3V6_VOLT_WINDOW;

      if ( error = lldriver->cmdAppOperCommand( ACMD41_ARG ); error != LLD::ERROR_NONE )
      {
        return error;
      }

      response     = lldriver->cpsmGetRespX( LLD::ResponseMailbox::RESPONSE_1 );
      device_ready = ( response & ACMD41_INIT_COMPLETE ) == ACMD41_INIT_COMPLETE;
      count++;
    }

    /*-------------------------------------------------------------------------
    Save reported card capacity information
    -------------------------------------------------------------------------*/
    cardInfo.CardType = ( response & ACMD41_HIGH_CAPACITY_CARD ) ? CardType::CARD_SDHC_SDXC : CardType::CARD_SDSC;

    return error;
  }


  /**
   * @brief Enables hardware support for 4-bit wide bus operation
   * @return LLD::ErrorType
   */
  LLD::ErrorType ThorImpl::busWidthEnable()
  {
    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    uint32_t       scr[ 2U ] = { 0U, 0U };
    LLD::ErrorType error;

    /*-------------------------------------------------------------------------
    Read the SCR register to determine SDCard capabilities
    -------------------------------------------------------------------------*/
    if ( error = lldriver->getStreamControlRegister( rca, scr ); error != LLD::ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Check if the card supports wide bus operation
    -------------------------------------------------------------------------*/
    if ( ( scr[ 1U ] & ACMD51_WIDE_BUS_SUPPORT ) != 0 )
    {
      /*-----------------------------------------------------------------------
      Indicate to the card the next command is application specific
      -----------------------------------------------------------------------*/
      if ( error = lldriver->cmdAppCommand( rca ); error != LLD::ERROR_NONE )
      {
        return error;
      }

      /*-----------------------------------------------------------------------
      Enable 4-bit wide bus operation by sending ACMD6.
      -----------------------------------------------------------------------*/
      if ( error = lldriver->cmdBusWidth( BusWidth::BUS_WIDTH_4BIT ); error != LLD::ERROR_NONE )
      {
        return error;
      }

      return LLD::ERROR_NONE;
    }
    else
    {
      return LLD::ERROR_REQUEST_NOT_APPLICABLE;
    }
  }


  /**
   * @brief Revert hardware back to single bit data bus operation
   * @return LLD::ErrorType
   */
  LLD::ErrorType ThorImpl::busWidthDisable()
  {
    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    uint32_t       scr[ 2U ] = { 0U, 0U };
    LLD::ErrorType error;

    /*-------------------------------------------------------------------------
    Read the SCR register to determine SDCard capabilities
    -------------------------------------------------------------------------*/
    if ( error = lldriver->getStreamControlRegister( rca, scr ); error != LLD::ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Check if the card supports single bit bus operation
    -------------------------------------------------------------------------*/
    if ( ( scr[ 1U ] & ACMD51_SINGLE_BUS_SUPPORT ) != 0 )
    {
      /*-----------------------------------------------------------------------
      Indicate to the card the next command is application specific
      -----------------------------------------------------------------------*/
      if ( error = lldriver->cmdAppCommand( rca ); error != LLD::ERROR_NONE )
      {
        return error;
      }

      /*-----------------------------------------------------------------------
      Enable 1-bit wide bus operation by sending ACMD6.
      -----------------------------------------------------------------------*/
      if ( error = lldriver->cmdBusWidth( BusWidth::BUS_WIDTH_1BIT ); error != LLD::ERROR_NONE )
      {
        return error;
      }

      return LLD::ERROR_NONE;
    }
    else
    {
      return LLD::ERROR_REQUEST_NOT_APPLICABLE;
    }
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
