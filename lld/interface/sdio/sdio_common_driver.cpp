/******************************************************************************
 *  File Name:
 *    sdio_common_driver.cpp
 *
 *  Description:
 *    Common driver for the STM32 SDIO peripheral
 *
 *  References:
 *    1. ATP Industrial Grade SD/SDHC Card Specification (Revision 2.6)
 *    2. Physical Layer Simplified Specification Version 9.00
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/sdio>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/sdio>

#if defined( THOR_SDIO ) && defined( TARGET_STM32F4 )
namespace Thor::LLD::SDIO
{
/*---------------------------------------------------------------------------
Literals
---------------------------------------------------------------------------*/
#define SDMMC_0TO7BITS        ( 0x000000FFU )
#define SDMMC_8TO15BITS       ( 0x0000FF00U )
#define SDMMC_16TO23BITS      ( 0x00FF0000U )
#define SDMMC_24TO31BITS      ( 0xFF000000U )
#define SDMMC_MAX_DATA_LENGTH ( 0x01FFFFFFU )

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t STATIC_CMD_FLAGS = STA_CCRCFAIL | STA_CTIMEOUT | STA_CMDREND | STA_CMDSENT;

  /**
   * @brief Command timeout in milliseconds
   */
  static constexpr uint32_t SDIO_CMD_TIMEOUT_MS = 5000u;

  /**
   * @brief Data timeout in milliseconds
   */
  static constexpr uint32_t SDIO_DATA_TIMEOUT_MS = 5000u;

  /**
   * @brief Max erase timeout in milliseconds
   */
  static constexpr uint32_t SDIO_MAX_ERASE_TIMEOUT_MS = 63000u;

  /**
   * @brief Timeout for STOP TRANSMISSION command in milliseconds
   */
  static constexpr uint32_t SDIO_STOP_TXFR_TIMEOUT = Chimera::Thread::TIMEOUT_BLOCK;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Calculates the HW clock divider setting for a given target bus frequency
   * @note Written using the formula from the STM32F4xx reference manual
   *
   * @param in_freq    Input clock frequency in Hz
   * @param out_freq   Target bus frequency in Hz
   * @return uint32_t HW register clock divider setting
   */
  static inline uint32_t calcClockDivider( const uint32_t in_freq, const uint32_t out_freq )
  {
    // Input constraints
    RT_DBG_ASSERT( in_freq > out_freq );
    RT_DBG_ASSERT( out_freq > 0 );

    // Calculate the divider and ensure it fits in the register field
    const uint32_t div = ( in_freq / out_freq ) - 2u;
    RT_DBG_ASSERT( ( div & CLKCR_CLKDIV_Msk ) == div );

    return div;
  }


  /**
   * @brief Waits for the CPSM to finish sending a command
   *
   * Potential exit conditions:
   *  - Timeout: A timeout occurred while waiting for a response
   *  - CRC Fail: Response CRC check failed
   *  - Success: Command was sent and response received successfully
   *
   * @param periph    Pointer to the SDIO peripheral register block
   * @param timeout   How long to wait for the command to finish
   * @return ErrorType
   */
  static ErrorType wait_cpsm_send_finish( const RegisterMap *const periph, const uint32_t timeout )
  {
    constexpr uint32_t abort_flags       = STA_CTIMEOUT | STA_CCRCFAIL | STA_CMDREND;
    constexpr uint32_t in_progress_flags = STA_CMDACT;
    const uint32_t     start             = Chimera::millis();
    uint32_t           status            = 0;

    do
    {
      if ( ( Chimera::millis() - start ) > timeout )
      {
        return ERROR_TIMEOUT;
      }

      status = STA_ALL::get( periph );

    } while ( ( ( status & abort_flags ) == 0 ) || ( ( status & in_progress_flags ) != 0 ) );

    return ERROR_NONE;
  }

  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), mResourceIdx( INVALID_RESOURCE_INDEX )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    /*-------------------------------------------------------------------------
    Get peripheral descriptor settings
    -------------------------------------------------------------------------*/
    mPeriph      = peripheral;
    mResourceIdx = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*-------------------------------------------------------------------------
    Handle the ISR configuration
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ mResourceIdx ] );
    INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIdx ] );
    INT::setPriority( Resource::IRQSignals[ mResourceIdx ], INT::SDIO_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    auto rcc = RCC::getPeriphClockCtrl();
    return rcc->reset( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIdx );
  }


  void Driver::clockEnable()
  {
    auto rcc = RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIdx );
  }


  void Driver::clockDisable()
  {
    auto rcc = RCC::getPeriphClockCtrl();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIdx );
  }


  void Driver::busClockEnable()
  {
    CLKEN::set( mPeriph, CLKCR_CLKEN );
  }


  void Driver::busClockDisable()
  {
    CLKEN::clear( mPeriph, CLKCR_CLKEN );
  }


  void Driver::enterCriticalSection()
  {
    INT::disableIRQ( Resource::IRQSignals[ mResourceIdx ] );
  }


  void Driver::exitCriticalSection()
  {
    INT::enableIRQ( Resource::IRQSignals[ mResourceIdx ] );
  }


  uint32_t Driver::getBusFrequency()
  {
    auto           rcc  = RCC::getCoreClockCtrl();
    const uint32_t pClk = rcc->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SDIO, reinterpret_cast<uintptr_t>( mPeriph ) );
    const uint32_t clkDiv = CLKDIV::get( mPeriph ) >> CLKCR_CLKDIV_Pos;

    return pClk / ( clkDiv + 2 );
  }


  Chimera::Status_t Driver::init()
  {
    /*-------------------------------------------------------------------------
    Reset the peripheral
    -------------------------------------------------------------------------*/
    reset();
    clockEnable();

    /*-------------------------------------------------------------------------
    Configure the clock divider to a default of 300KHz
    -------------------------------------------------------------------------*/
    auto           rcc  = RCC::getCoreClockCtrl();
    const uint32_t pClk = rcc->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SDIO, reinterpret_cast<uintptr_t>( mPeriph ) );
    const uint32_t clkDiv = calcClockDivider( pClk, 400000u );

    CLKDIV::set( mPeriph, clkDiv << CLKCR_CLKDIV_Pos );

    /*-------------------------------------------------------------------------
    Set remainder of the clock control register
    -------------------------------------------------------------------------*/
    HWFCEN::clear( mPeriph, CLKCR_HWFC_EN );    // Disable HW flow control
    PWRSAV::set( mPeriph, CLKCR_PWRSAV );       // Enable power saving mode
    NEGEDGE::set( mPeriph, CLKCR_NEGEDGE );     // Data is changed on the falling edge
    WIDBUS::set( mPeriph, 0 );                  // 1-bit bus width
    BYPASS::clear( mPeriph, CLKCR_BYPASS );     // Disable clock bypass
    CLKEN::clear( mPeriph, CLKCR_CLKEN );       // Disable the clock output

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setPowerStateOn()
  {
    PWRCTRL::set( mPeriph, POWER_PWRCTRL_ON );
    Chimera::blockDelayMilliseconds( 2 );

    return Chimera::Status::OK;
  }


  void Driver::cpsmPutCmd( const CPSMCommand &cmd )
  {
    /*-------------------------------------------------------------------------
    Prepare the command register
    -------------------------------------------------------------------------*/
    CMDARG::set( mPeriph, cmd.Argument );
    CMDINDEX::set( mPeriph, cmd.CmdIndex );
    WAITRESP::set( mPeriph, cmd.Response );
    WAITINT::set( mPeriph, cmd.WaitForInterrupt );

    /*-------------------------------------------------------------------------
    Start the transfer
    -------------------------------------------------------------------------*/
    CPSMEN::set( mPeriph, cmd.CPSM );
  }


  ErrorType Driver::cmdAppCommand( const uint32_t Argument )
  {
    /*-------------------------------------------------------------------------
    Build and send the command
    -------------------------------------------------------------------------*/
    CPSMCommand cmd;

    cmd.Argument         = Argument;
    cmd.CmdIndex         = CMD_APP_CMD;
    cmd.Response         = CMD_RESPONSE_SHORT;
    cmd.WaitForInterrupt = CMD_WAIT_NO;
    cmd.CPSM             = CMD_CPSM_ENABLE;

    cpsmPutCmd( cmd );

    /*-------------------------------------------------------------------------
    Return the command response
    -------------------------------------------------------------------------*/
    return getCmdResp1( cmd.CmdIndex, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdAppOperCommand( const uint32_t Argument )
  {
    /*-------------------------------------------------------------------------
    Build and send the command
    -------------------------------------------------------------------------*/
    CPSMCommand cmd;

    cmd.Argument         = Argument;
    cmd.CmdIndex         = CMD_SD_APP_OP_COND;
    cmd.Response         = CMD_RESPONSE_SHORT;
    cmd.WaitForInterrupt = CMD_WAIT_NO;
    cmd.CPSM             = CMD_CPSM_ENABLE;

    cpsmPutCmd( cmd );

    /*-------------------------------------------------------------------------
    Return the command response
    -------------------------------------------------------------------------*/
    return getCmdResp3();
  }


  ErrorType Driver::cmdBlockLength( const uint32_t blockSize )
  {
    /*-------------------------------------------------------------------------
    Build and send the command
    -------------------------------------------------------------------------*/
    CPSMCommand cmd;

    cmd.Argument         = blockSize;
    cmd.CmdIndex         = CMD_SET_BLOCKLEN;
    cmd.Response         = CMD_RESPONSE_SHORT;
    cmd.WaitForInterrupt = CMD_WAIT_NO;
    cmd.CPSM             = CMD_CPSM_ENABLE;

    cpsmPutCmd( cmd );

    /*-------------------------------------------------------------------------
    Return the command response
    -------------------------------------------------------------------------*/
    return getCmdResp1( cmd.CmdIndex, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdBusWidth( const Chimera::SDIO::BusWidth width )
  {
    using namespace Chimera::SDIO;

    /*-------------------------------------------------------------------------
    Build and send the command
    -------------------------------------------------------------------------*/
    CPSMCommand cmd;

    cmd.CmdIndex         = CMD_SD_APP_SET_BUSWIDTH;
    cmd.Response         = CMD_RESPONSE_SHORT;
    cmd.WaitForInterrupt = CMD_WAIT_NO;
    cmd.CPSM             = CMD_CPSM_ENABLE;

    switch ( width )
    {
      case BusWidth::BUS_WIDTH_1BIT:
        cmd.Argument = 0u;
        break;

      case BusWidth::BUS_WIDTH_4BIT:
        cmd.Argument = CLKCR_WIDBUS_0;
        break;

      default:
        return ErrorType::ERROR_INVALID_PARAMETER;
    }

    cpsmPutCmd( cmd );

    /*-------------------------------------------------------------------------
    Return the command response
    -------------------------------------------------------------------------*/
    if ( auto error = getCmdResp1( cmd.CmdIndex, SDIO_CMD_TIMEOUT_MS ); error != ErrorType::ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Update the bus width setting in the peripheral
    -------------------------------------------------------------------------*/
    WIDBUS::set( mPeriph, cmd.Argument );
    return ErrorType::ERROR_NONE;
  }


  ErrorType Driver::cmdGoIdleState()
  {
    /*-------------------------------------------------------------------------
    Send the command
    -------------------------------------------------------------------------*/
    CPSMCommand cmd;
    cmd.Argument         = 0U;
    cmd.CmdIndex         = CMD_GO_IDLE_STATE;
    cmd.Response         = CMD_RESPONSE_NO;
    cmd.WaitForInterrupt = CMD_WAIT_NO;
    cmd.CPSM             = CMD_CPSM_ENABLE;

    cpsmPutCmd( cmd );

    /*-------------------------------------------------------------------------
    Wait for the command to complete
    -------------------------------------------------------------------------*/
    const uint32_t start = Chimera::millis();

    do
    {
      if ( ( Chimera::millis() - start ) > SDIO_CMD_TIMEOUT_MS )
      {
        return ERROR_TIMEOUT;
      }

    } while ( ( STA_ALL::get( mPeriph ) & STA_CMDSENT ) != STA_CMDSENT );

    /*-------------------------------------------------------------------------
    Clear all static flags
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );
    return ERROR_NONE;
  }


  ErrorType Driver::cmdOperCond()
  {
    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    /* Operating Conditions Argument: Table 6-18
      - [31:12]: Reserved (shall be set to '0')
      -  [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
      -   [7:0]: Check Pattern (recommended 0xAA) */
    static constexpr uint32_t ARGUMENT = 0x000001AAu;

    /*-------------------------------------------------------------------------
    Send CMD8 to verify SD card interface operating condition
    Build and send the command
    -------------------------------------------------------------------------*/
    CPSMCommand cmd;
    cmd.Argument         = ARGUMENT;
    cmd.CmdIndex         = CMD_HS_SEND_EXT_CSD;
    cmd.Response         = CMD_RESPONSE_SHORT;
    cmd.WaitForInterrupt = CMD_WAIT_NO;
    cmd.CPSM             = CMD_CPSM_ENABLE;

    cpsmPutCmd( cmd );

    /*-------------------------------------------------------------------------
    Return the command response
    -------------------------------------------------------------------------*/
    return getCmdResp7();
  }


  ErrorType Driver::cmdSendSCR()
  {
    /*-------------------------------------------------------------------------
    Send CMD51 to read the SD Configuration Register (SCR)
    -------------------------------------------------------------------------*/
    CPSMCommand cmd;
    cmd.Argument         = 0u;
    cmd.CmdIndex         = CMD_SD_APP_SEND_SCR;
    cmd.Response         = CMD_RESPONSE_SHORT;
    cmd.WaitForInterrupt = CMD_WAIT_NO;
    cmd.CPSM             = CMD_CPSM_ENABLE;

    cpsmPutCmd( cmd );

    /*-------------------------------------------------------------------------
    Return the command response
    -------------------------------------------------------------------------*/
    return getCmdResp1( cmd.CmdIndex, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::getCmdResp1( uint8_t SD_CMD, uint32_t Timeout )
  {
    using namespace Chimera::SDIO;

    /*-------------------------------------------------------------------------
    Do timeout on flag updates
    -------------------------------------------------------------------------*/
    if ( auto error = wait_cpsm_send_finish( mPeriph, SDIO_CMD_TIMEOUT_MS ); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Handle error codes
    -------------------------------------------------------------------------*/
    if ( CTIMEOUT::get( mPeriph ) == STA_CTIMEOUT )
    {
      CTIMEOUTC::set( mPeriph, ICR_CTIMEOUTC );
      return ERROR_CMD_RSP_TIMEOUT;
    }
    else if ( CCRCFAIL::get( mPeriph ) == STA_CCRCFAIL )
    {
      CCRCFAILC::set( mPeriph, ICR_CCRCFAILC );
      return ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );

    /* Check response received is of desired command */
    if ( CMDRESP::get( mPeriph ) != SD_CMD )
    {
      return ERROR_CMD_CRC_FAIL;
    }

    /* We have received response, retrieve it for analysis  */
    auto response_r1 = cpsmGetResponse( ResponseMailbox::RESPONSE_1 );

    if ( ( response_r1 & OCR_ERRORBITS ) == 0 )
    {
      return ERROR_NONE;
    }
    else if ( ( response_r1 & OCR_ADDR_OUT_OF_RANGE ) == OCR_ADDR_OUT_OF_RANGE )
    {
      return ERROR_ADDR_OUT_OF_RANGE;
    }
    else if ( ( response_r1 & OCR_ADDR_MISALIGNED ) == OCR_ADDR_MISALIGNED )
    {
      return ERROR_ADDR_MISALIGNED;
    }
    else if ( ( response_r1 & OCR_BLOCK_LEN_ERR ) == OCR_BLOCK_LEN_ERR )
    {
      return ERROR_BLOCK_LEN_ERR;
    }
    else if ( ( response_r1 & OCR_ERASE_SEQ_ERR ) == OCR_ERASE_SEQ_ERR )
    {
      return ERROR_ERASE_SEQ_ERR;
    }
    else if ( ( response_r1 & OCR_BAD_ERASE_PARAM ) == OCR_BAD_ERASE_PARAM )
    {
      return ERROR_BAD_ERASE_PARAM;
    }
    else if ( ( response_r1 & OCR_WRITE_PROT_VIOLATION ) == OCR_WRITE_PROT_VIOLATION )
    {
      return ERROR_WRITE_PROT_VIOLATION;
    }
    else if ( ( response_r1 & OCR_LOCK_UNLOCK_FAILED ) == OCR_LOCK_UNLOCK_FAILED )
    {
      return ERROR_LOCK_UNLOCK_FAILED;
    }
    else if ( ( response_r1 & OCR_COM_CRC_FAILED ) == OCR_COM_CRC_FAILED )
    {
      return ERROR_COM_CRC_FAILED;
    }
    else if ( ( response_r1 & OCR_ILLEGAL_CMD ) == OCR_ILLEGAL_CMD )
    {
      return ERROR_ILLEGAL_CMD;
    }
    else if ( ( response_r1 & OCR_CARD_ECC_FAILED ) == OCR_CARD_ECC_FAILED )
    {
      return ERROR_CARD_ECC_FAILED;
    }
    else if ( ( response_r1 & OCR_CC_ERROR ) == OCR_CC_ERROR )
    {
      return ERROR_CC_ERR;
    }
    else if ( ( response_r1 & OCR_STREAM_READ_UNDERRUN ) == OCR_STREAM_READ_UNDERRUN )
    {
      return ERROR_STREAM_READ_UNDERRUN;
    }
    else if ( ( response_r1 & OCR_STREAM_WRITE_OVERRUN ) == OCR_STREAM_WRITE_OVERRUN )
    {
      return ERROR_STREAM_WRITE_OVERRUN;
    }
    else if ( ( response_r1 & OCR_CID_CSD_OVERWRITE ) == OCR_CID_CSD_OVERWRITE )
    {
      return ERROR_CID_CSD_OVERWRITE;
    }
    else if ( ( response_r1 & OCR_WP_ERASE_SKIP ) == OCR_WP_ERASE_SKIP )
    {
      return ERROR_WP_ERASE_SKIP;
    }
    else if ( ( response_r1 & OCR_CARD_ECC_DISABLED ) == OCR_CARD_ECC_DISABLED )
    {
      return ERROR_CARD_ECC_DISABLED;
    }
    else if ( ( response_r1 & OCR_ERASE_RESET ) == OCR_ERASE_RESET )
    {
      return ERROR_ERASE_RESET;
    }
    else if ( ( response_r1 & OCR_AKE_SEQ_ERROR ) == OCR_AKE_SEQ_ERROR )
    {
      return ERROR_AKE_SEQ_ERR;
    }
    else
    {
      return ERROR_GENERAL_UNKNOWN_ERR;
    }
  }


  ErrorType Driver::getCmdResp3()
  {
    /*-------------------------------------------------------------------------
    Do timeout on flag updates
    -------------------------------------------------------------------------*/
    if ( auto error = wait_cpsm_send_finish( mPeriph, SDIO_CMD_TIMEOUT_MS ); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Parse the error code. CRC error is not checked for R3 response, as this is
    only used for the CMD_SD_APP_OP_COND command, which does not have a CRC.
    -------------------------------------------------------------------------*/
    if ( CTIMEOUT::get( mPeriph ) == STA_CTIMEOUT )
    {
      CTIMEOUTC::set( mPeriph, ICR_CTIMEOUTC );
      return ERROR_CMD_RSP_TIMEOUT;
    }
    else
    {
      /* Clear all the static flags */
      ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );
    }

    return ERROR_NONE;
  }


  ErrorType Driver::getCmdResp7()
  {
    /*-------------------------------------------------------------------------
    Do timeout on flag updates
    -------------------------------------------------------------------------*/
    if ( auto error = wait_cpsm_send_finish( mPeriph, SDIO_CMD_TIMEOUT_MS ); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Parse the error code
    -------------------------------------------------------------------------*/
    if ( CTIMEOUT::get( mPeriph ) == STA_CTIMEOUT )
    {
      CTIMEOUTC::set( mPeriph, ICR_CTIMEOUTC );
      return ERROR_CMD_RSP_TIMEOUT;
    }
    else if ( CCRCFAIL::get( mPeriph ) == STA_CCRCFAIL )
    {
      CCRCFAILC::set( mPeriph, ICR_CCRCFAILC );
      return ERROR_CMD_CRC_FAIL;
    }
    else if ( ( STA_ALL::get( mPeriph ) & STA_CMDREND ) == STA_CMDREND )
    {
      CMDRENDC::set( mPeriph, ICR_CMDRENDC );
    }

    return ERROR_NONE;
  }


  uint32_t Driver::cpsmGetResponse( const ResponseMailbox which )
  {
    /*-------------------------------------------------------------------------
    Find the address of the response register and read it
    -------------------------------------------------------------------------*/
    auto response_register = ( uint32_t )( &( mPeriph->RESP1 ) ) + EnumValue( which );
    return ( *( volatile uint32_t * )response_register );
  }


  void Driver::dpsmConfigure( const DPSMConfig &config )
  {
    /*-------------------------------------------------------------------------
    Configure the DPSM transaction parameters
    -------------------------------------------------------------------------*/
    DATATIME::set( mPeriph, config.DataTimeOut );
    DATALENGTH::set( mPeriph, config.DataLength );
    DBLOCKSIZE::set( mPeriph, config.DataBlockSize );
    DTMODE::set( mPeriph, config.TransferMode );
    DTDIR::set( mPeriph, config.TransferDir );

    /*-------------------------------------------------------------------------
    Finally, set the DPSM overall state
    -------------------------------------------------------------------------*/
    DTEN::set( mPeriph, config.DPSM );
  }


  ErrorType Driver::getStreamControlRegister( uint32_t *const pSCR )
  {
    DPSMConfig config;
    ErrorType  error;
    uint32_t   tickstart     = Chimera::millis();
    uint32_t   index         = 0U;
    uint32_t   tempscr[ 2U ] = { 0U, 0U };
    uint32_t  *scr           = pSCR;

    /* Set Block Size To 8 Bytes */
    if ( error = cmdBlockLength( 8U ); error != ERROR_NONE )
    {
      return error;
    }

    /* Send CMD55 APP_CMD with argument as card's RCA */
    if ( error = cmdAppCommand( 0u ); error != ERROR_NONE )
    {
      return error;
    }

    config.DataTimeOut   = 5000;
    config.DataLength    = 8U;
    config.DataBlockSize = DCTRL_DATABLOCK_SIZE_8B;
    config.TransferDir   = TRANSFER_DIR_TO_SDIO;
    config.TransferMode  = TRANSFER_MODE_BLOCK;
    config.DPSM          = DPSM_ENABLE;
    dpsmConfigure( config );

    /* Send ACMD51 SD_APP_SEND_SCR */
    if ( error = cmdSendSCR(); error != ERROR_NONE )
    {
      return error;
    }

    while ( STA_ALL::get( mPeriph ) & ( STA_RXOVERR | STA_DCRCFAIL | STA_DTIMEOUT ) == 0 )
    {
      if ( RXDAVL::get( mPeriph ) )
      {
        *( tempscr + index ) = mPeriph->FIFO;
        index++;
      }
      else if ( RXACT::get( mPeriph ) != STA_RXACT )
      {
        /* No reception in-progress anymore */
        break;
      }

      if ( ( Chimera::millis() - tickstart ) >= SDIO_DATA_TIMEOUT_MS )
      {
        return ErrorType::ERROR_TIMEOUT;
      }
    }

    if ( DTIMEOUT::get( mPeriph ) == STA_DTIMEOUT )
    {
      DTIMEOUTC::set( mPeriph, STA_DTIMEOUT );
      return ErrorType::ERROR_DATA_TIMEOUT;
    }
    else if ( DCRCFAIL::get( mPeriph ) == STA_DCRCFAIL )
    {
      DCRCFAILC::set( mPeriph, STA_DCRCFAIL );
      return ErrorType::ERROR_DATA_CRC_FAIL;
    }
    else if ( RXOVERR::get( mPeriph ) == STA_RXOVERR )
    {
      RXOVERRC::set( mPeriph, STA_RXOVERR );
      return ErrorType::ERROR_RX_OVERRUN;
    }
    else
    {
      /* Clear all the static flags */
      ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );

      *scr = ( ( ( tempscr[ 1 ] & SDMMC_0TO7BITS ) << 24 ) | ( ( tempscr[ 1 ] & SDMMC_8TO15BITS ) << 8 ) |
               ( ( tempscr[ 1 ] & SDMMC_16TO23BITS ) >> 8 ) | ( ( tempscr[ 1 ] & SDMMC_24TO31BITS ) >> 24 ) );
      scr++;
      *scr = ( ( ( tempscr[ 0 ] & SDMMC_0TO7BITS ) << 24 ) | ( ( tempscr[ 0 ] & SDMMC_8TO15BITS ) << 8 ) |
               ( ( tempscr[ 0 ] & SDMMC_16TO23BITS ) >> 8 ) | ( ( tempscr[ 0 ] & SDMMC_24TO31BITS ) >> 24 ) );
    }

    return ErrorType::ERROR_NONE;
  }


  void Driver::IRQHandler()
  {
  }
}    // namespace Thor::LLD::SDIO
#endif /* THOR_SDIO && TARGET_STM32F4 */
