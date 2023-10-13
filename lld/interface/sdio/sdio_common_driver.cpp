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
#include <Chimera/dma>
#include <Chimera/sdio>
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/sdio>

#if defined( THOR_SDIO ) && defined( TARGET_STM32F4 )
namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Local Types
  ---------------------------------------------------------------------------*/
  struct ControlBlock
  {
    volatile ErrorType      txfrError;  /**< Final determined error */
    volatile uint32_t       txfrStatus; /**< Recorded value of status register termination */
    Chimera::DMA::RequestId txfrId;     /**< DMA transaction request ID */
  };

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  /**
   * @brief Collection of static command flags
   */
  static constexpr uint32_t STATIC_CMD_FLAGS = STA_CCRCFAIL | STA_CTIMEOUT | STA_CMDREND | STA_CMDSENT;

  /**
   * @brief Interrupt flags to enable for TX
   */
  static constexpr uint32_t TX_ISR_MASK_FLAGS =
      MASK_TXUNDERRIE | MASK_DCRCFAILIE | MASK_DTIMEOUTIE | MASK_DBCKENDIE | MASK_DATAENDIE;
  static constexpr uint32_t TX_ISR_CLEAR_FLAGS  = ICR_TXUNDERRC | ICR_DCRCFAILC | ICR_DTIMEOUTC | ICR_DBCKENDC | ICR_DATAENDC;
  static constexpr uint32_t TX_ISR_STATUS_FLAGS = STA_TXUNDERR | STA_DCRCFAIL | STA_DTIMEOUT | STA_DBCKEND | STA_DATAEND;

  /**
   * @brief Interrupt flags to enable for RX
   */
  static constexpr uint32_t RX_ISR_MASK_FLAGS   = MASK_RXOVERRIE | MASK_DBCKENDIE | MASK_DCRCFAILIE | MASK_DTIMEOUTIE;
  static constexpr uint32_t RX_ISR_CLEAR_FLAGS  = ICR_RXOVERRC | ICR_DBCKENDC | ICR_DCRCFAILC | ICR_DTIMEOUTC;
  static constexpr uint32_t RX_ISR_STATUS_FLAGS = STA_RXOVERR | STA_DBCKEND | STA_DCRCFAIL | STA_DTIMEOUT;

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

  /**
   * @brief Block size for data transfers
   */
  static constexpr uint32_t SDIO_BLOCK_SIZE = 512u;

  /**
   * @brief Command configuration register settings
   */
  static constexpr etl::array<uint16_t, CommandType::CMD_TOTAL_COUNT> CmdRegCfg = {
    ( CMD_GO_IDLE_STATE | CMD_RESPONSE_NO | CMD_WAIT_NO | CMD_CPSM_ENABLE ),          /**<  0 -- CMD_GO_IDLE_STATE */
    ( 0 ),                                                                            /**<  1 -- CMD_SEND_OP_COND */
    ( CMD_ALL_SEND_CID | CMD_RESPONSE_LONG | CMD_WAIT_NO | CMD_CPSM_ENABLE ),         /**<  2 -- CMD_ALL_SEND_CID */
    ( CMD_SET_REL_ADDR | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),        /**<  3 -- CMD_SET_REL_ADDR */
    ( 0 ),                                                                            /**<  4 -- CMD_SET_DSR */
    ( 0 ),                                                                            /**<  5 -- CMD_SDMMC_SEN_OP_COND */
    ( CMD_SD_APP_SET_BUSWIDTH | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ), /**<  6 -- CMD_SD_APP_SET_BUSWIDTH */
    ( CMD_SEL_DESEL_CARD | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),      /**<  7 -- CMD_SEL_DESEL_CARD */
    ( CMD_HS_SEND_EXT_CSD | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),     /**<  8 -- CMD_HS_SEND_EXT_CSD */
    ( CMD_SEND_CSD | CMD_RESPONSE_LONG | CMD_WAIT_NO | CMD_CPSM_ENABLE ),             /**<  9 -- CMD_SEND_CSD */
    ( 0 ),                                                                            /**< 10 -- CMD_SEND_CID */
    ( 0 ),                                                                            /**< 11 -- CMD_READ_DAT_UNTIL_STOP */
    ( CMD_STOP_TRANSMISSION | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),   /**< 12 -- CMD_STOP_TRANSMISSION */
    ( CMD_SEND_STATUS | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),         /**< 13 -- CMD_SEND_STATUS */
    ( 0 ),                                                                            /**< 14 -- CMD_HS_BUSTEST_READ */
    ( 0 ),                                                                            /**< 15 -- ??? */
    ( CMD_SET_BLOCKLEN | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),        /**< 16 -- CMD_SET_BLOCKLEN */
    ( CMD_READ_SINGLE_BLOCK | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),   /**< 17 -- CMD_READ_SINGLE_BLOCK */
    ( CMD_READ_MULT_BLOCK | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),     /**< 18 -- CMD_READ_MULT_BLOCK */
    ( 0 ),                                                                            /**< 19 -- CMD_HS_BUSTEST_WRITE */
    ( 0 ),                                                                            /**< 20 -- CMD_WRITE_DAT_UNTIL_STOP */
    ( 0 ),                                                                            /**< 21 -- ??? */
    ( 0 ),                                                                            /**< 22 -- ??? */
    ( 0 ),                                                                            /**< 23 -- CMD_SET_BLOCK_COUNT */
    ( CMD_WRITE_SINGLE_BLOCK | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),  /**< 24 -- CMD_WRITE_SINGLE_BLOCK */
    ( CMD_WRITE_MULT_BLOCK | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),    /**< 25 -- CMD_WRITE_MULT_BLOCK */
    ( 0 ),                                                                            /**< 26 -- CMD_PROG_CID */
    ( 0 ),                                                                            /**< 27 -- CMD_PROG_CSD */
    ( 0 ),                                                                            /**< 28 -- CMD_SET_WRITE_PROT */
    ( 0 ),                                                                            /**< 29 -- CMD_CLR_WRITE_PROT */
    ( 0 ),                                                                            /**< 30 -- CMD_SEND_WRITE_PROT */
    ( 0 ),                                                                            /**< 31 -- ??? */
    ( CMD_SD_ERASE_GRP_START | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),  /**< 32 -- CMD_SD_ERASE_GRP_START */
    ( CMD_SD_ERASE_GRP_END | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),    /**< 33 -- CMD_SD_ERASE_GRP_END */
    ( 0 ),                                                                            /**< 34 -- ??? */
    ( 0 ),                                                                            /**< 35 -- CMD_ERASE_GRP_START */
    ( 0 ),                                                                            /**< 36 -- CMD_ERASE_GRP_END */
    ( 0 ),                                                                            /**< 37 -- ??? */
    ( CMD_ERASE | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),               /**< 38 -- CMD_ERASE */
    ( 0 ),                                                                            /**< 39 -- CMD_FAST_IO */
    ( 0 ),                                                                            /**< 40 -- CMD_GO_IRQ_STATE */
    ( CMD_SD_APP_OP_COND | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),      /**< 41 -- CMD_SD_APP_OP_COND */
    ( 0 ),                                                                            /**< 42 -- CMD_LOCK_UNLOCK */
    ( 0 ),                                                                            /**< 43 -- ??? */
    ( 0 ),                                                                            /**< 44 -- ??? */
    ( 0 ),                                                                            /**< 45 -- ??? */
    ( 0 ),                                                                            /**< 46 -- ??? */
    ( 0 ),                                                                            /**< 47 -- ??? */
    ( 0 ),                                                                            /**< 48 -- ??? */
    ( 0 ),                                                                            /**< 49 -- ??? */
    ( 0 ),                                                                            /**< 50 -- ??? */
    ( CMD_SD_APP_SEND_SCR | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),     /**< 51 -- CMD_SD_APP_SEND_SCR */
    ( 0 ),                                                                            /**< 52 -- ??? */
    ( 0 ),                                                                            /**< 53 -- ??? */
    ( 0 ),                                                                            /**< 54 -- ??? */
    ( CMD_APP_CMD | CMD_RESPONSE_SHORT | CMD_WAIT_NO | CMD_CPSM_ENABLE ),             /**< 55 -- CMD_APP_CMD */
    ( 0 ),                                                                            /**< 56 -- CMD_GEN_CMD */
    ( 0 ),                                                                            /**< 57 -- ??? */
    ( 0 ),                                                                            /**< 58 -- ??? */
    ( 0 ),                                                                            /**< 59 -- ??? */
    ( 0 ),                                                                            /**< 60 -- ??? */
    ( 0 ),                                                                            /**< 61 -- ??? */
    ( 0 ),                                                                            /**< 62 -- ??? */
    ( 0 ),                                                                            /**< 63 -- ??? */
    ( 0 ),                                                                            /**< 64 -- CMD_NO_CMD */
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static ControlBlock s_ctrl_blk[ NUM_SDIO_PERIPHS ];

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
  static inline uint32_t calc_clock_div( const uint32_t in_freq, const uint32_t out_freq )
  {
    // Input constraints
    RT_DBG_ASSERT( in_freq >= out_freq );
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
  Driver::Driver() : mPeriph( nullptr ), mResourceIndex( INVALID_RESOURCE_INDEX )
  {
  }


  Driver::~Driver()
  {
  }

  /*---------------------------------------------------------------------------
  Peripheral Control Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    /*-------------------------------------------------------------------------
    Get peripheral descriptor settings
    -------------------------------------------------------------------------*/
    mPeriph        = peripheral;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*-------------------------------------------------------------------------
    Handle the ISR configuration
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    INT::setPriority( Resource::IRQSignals[ mResourceIndex ], INT::SDIO_IT_PREEMPT_PRIORITY, 0u );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::reset()
  {
    /*-------------------------------------------------------------------------
    Reset the AsyncIO driver
    -------------------------------------------------------------------------*/
    resetAIO();

    /*-------------------------------------------------------------------------
    Reset the peripheral HW via clock control
    -------------------------------------------------------------------------*/
    auto rcc = RCC::getPeriphClockCtrl();
    return rcc->reset( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIndex );
  }


  void Driver::clockEnable()
  {
    auto rcc = RCC::getPeriphClockCtrl();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIndex );
  }


  void Driver::clockDisable()
  {
    auto rcc = RCC::getPeriphClockCtrl();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_SDIO, mResourceIndex );
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
    INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  void Driver::exitCriticalSection()
  {
    INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );
  }


  uint32_t Driver::getBusFrequency()
  {
    auto           rcc  = RCC::getCoreClockCtrl();
    const uint32_t pClk = rcc->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SDIO, reinterpret_cast<uintptr_t>( mPeriph ) );
    const uint32_t clkDiv = CLKDIV::get( mPeriph ) >> CLKCR_CLKDIV_Pos;

    return pClk / ( clkDiv + 2 );
  }


  ErrorType Driver::setBusFrequency( const uint32_t freq )
  {
    auto           rcc  = RCC::getCoreClockCtrl();
    const uint32_t pClk = rcc->getPeriphClock( Chimera::Peripheral::Type::PERIPH_SDIO, reinterpret_cast<uintptr_t>( mPeriph ) );
    const uint32_t clkDiv = calc_clock_div( pClk, freq );

    CLKDIV::set( mPeriph, clkDiv << CLKCR_CLKDIV_Pos );
    return ERROR_NONE;
  }


  Chimera::Status_t Driver::init()
  {
    using namespace Chimera::DMA;

    /*-------------------------------------------------------------------------
    Reset the peripheral
    -------------------------------------------------------------------------*/
    reset();
    clockEnable();

    /*-------------------------------------------------------------------------
    Ensure the AsyncIO driver is ready
    -------------------------------------------------------------------------*/
    initAIO();

    /*-------------------------------------------------------------------------
    Configure the clock divider to a default of ~300KHz
    -------------------------------------------------------------------------*/
    setBusFrequency( 300000u );

    /*-------------------------------------------------------------------------
    Set remainder of the clock control register
    -------------------------------------------------------------------------*/
    HWFCEN::clear( mPeriph, CLKCR_HWFC_EN );    // Disable HW flow control
    PWRSAV::clear( mPeriph, CLKCR_PWRSAV );     // Disable power saving mode
    NEGEDGE::set( mPeriph, CLKCR_NEGEDGE );     // Data is changed on the falling edge
    WIDBUS::set( mPeriph, 0 );                  // 1-bit bus width
    BYPASS::clear( mPeriph, CLKCR_BYPASS );     // Disable clock bypass
    CLKEN::clear( mPeriph, CLKCR_CLKEN );       // Disable the clock output

    /*-------------------------------------------------------------------------
    Initialize DMA RX Pipe
    -------------------------------------------------------------------------*/
    PipeConfig rxCfg;
    rxCfg.clear();

    rxCfg.srcAlignment  = Alignment::WORD;
    rxCfg.dstAlignment  = Alignment::WORD;
    rxCfg.direction     = Direction::PERIPH_TO_MEMORY;
    rxCfg.mode          = Mode::PERIPHERAL;
    rxCfg.priority      = Priority::VERY_HIGH;
    rxCfg.resourceIndex = Thor::LLD::DMA::DMA2_STREAM3_RESOURCE_INDEX;
    rxCfg.channel       = static_cast<size_t>( Thor::LLD::DMA::Channel::CHANNEL_4 );
    rxCfg.fifoMode      = FifoMode::DIRECT_DISABLE;
    rxCfg.threshold     = FifoThreshold::FULL;
    rxCfg.burstSize     = BurstSize::BURST_SIZE_4;
    rxCfg.persistent    = true;
    rxCfg.periphAddr    = reinterpret_cast<uintptr_t>( &mPeriph->FIFO );

    mRXDMAPipeId = Chimera::DMA::constructPipe( rxCfg );
    RT_DBG_ASSERT( mRXDMAPipeId != Chimera::DMA::INVALID_REQUEST );

    /*-------------------------------------------------------------------------
    Initialize DMA TX Pipe
    -------------------------------------------------------------------------*/
    PipeConfig txCfg;
    txCfg.clear();

    txCfg.srcAlignment  = Alignment::WORD;
    txCfg.dstAlignment  = Alignment::WORD;
    txCfg.direction     = Direction::MEMORY_TO_PERIPH;
    txCfg.mode          = Mode::PERIPHERAL;
    txCfg.priority      = Priority::VERY_HIGH;
    txCfg.resourceIndex = Thor::LLD::DMA::DMA2_STREAM6_RESOURCE_INDEX;
    txCfg.channel       = static_cast<size_t>( Thor::LLD::DMA::Channel::CHANNEL_4 );
    txCfg.fifoMode      = FifoMode::DIRECT_DISABLE;
    txCfg.threshold     = FifoThreshold::FULL;
    txCfg.burstSize     = BurstSize::BURST_SIZE_4;
    txCfg.persistent    = true;
    txCfg.periphAddr    = reinterpret_cast<uintptr_t>( &mPeriph->FIFO );

    mTXDMAPipeId = Chimera::DMA::constructPipe( txCfg );
    RT_DBG_ASSERT( mTXDMAPipeId != Chimera::DMA::INVALID_REQUEST );
    RT_DBG_ASSERT( mTXDMAPipeId != mRXDMAPipeId );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setPowerStateOn()
  {
    PWRCTRL::set( mPeriph, POWER_PWRCTRL_ON );
    Chimera::blockDelayMilliseconds( 2 );

    return Chimera::Status::OK;
  }


  uint32_t Driver::cpsmGetRespX( const ResponseMailbox which ) const
  {
    /*-------------------------------------------------------------------------
    Find the address of the response register and read it
    -------------------------------------------------------------------------*/
    auto response_register = ( uint32_t )( &( mPeriph->RESP1 ) ) + EnumValue( which );
    return ( *( volatile uint32_t * )response_register );
  }


  /*---------------------------------------------------------------------------
  Command Management
  ---------------------------------------------------------------------------*/
  ErrorType Driver::cmdAppCommand( const uint16_t rca )
  {
    /*-------------------------------------------------------------------------
    Send CMD55 to indicate the next command is an application specific command
    -------------------------------------------------------------------------*/
    mPeriph->ARG = static_cast<uint32_t>( rca ) << 16u;
    mPeriph->CMD = CmdRegCfg[ CMD_APP_CMD ];

    return getCmdResp1( CMD_APP_CMD, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdAppOperCommand( const uint32_t Argument )
  {
    /*-------------------------------------------------------------------------
    Send ACMD41 to initialize the SD card
    -------------------------------------------------------------------------*/
    mPeriph->ARG = Argument;
    mPeriph->CMD = CmdRegCfg[ CMD_SD_APP_OP_COND ];

    return getCmdResp3();
  }


  ErrorType Driver::cmdBlockLength( const uint32_t blockSize )
  {
    /*-------------------------------------------------------------------------
    Send CMD16 to set the block length
    -------------------------------------------------------------------------*/
    mPeriph->ARG = blockSize;
    mPeriph->CMD = CmdRegCfg[ CMD_SET_BLOCKLEN ];

    return getCmdResp1( CMD_SET_BLOCKLEN, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdBusWidth( const Chimera::SDIO::BusWidth width )
  {
    /*-------------------------------------------------------------------------
    Send CMD6 to set the bus width
    -------------------------------------------------------------------------*/
    switch ( width )
    {
      case Chimera::SDIO::BusWidth::BUS_WIDTH_1BIT:
        mPeriph->ARG = 0u;
        break;

      case Chimera::SDIO::BusWidth::BUS_WIDTH_4BIT:
        mPeriph->ARG = CLKCR_WIDBUS_0;
        break;

      default:
        return ErrorType::ERROR_INVALID_PARAMETER;
    }

    mPeriph->CMD = CmdRegCfg[ CMD_SD_APP_SET_BUSWIDTH ];

    if ( auto error = getCmdResp1( CMD_SD_APP_SET_BUSWIDTH, SDIO_CMD_TIMEOUT_MS ); error != ErrorType::ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Update the bus width setting in the peripheral if the command succeeded
    -------------------------------------------------------------------------*/
    WIDBUS::set( mPeriph, mPeriph->ARG );
    return ErrorType::ERROR_NONE;
  }


  ErrorType Driver::cmdErase()
  {
    /*-------------------------------------------------------------------------
    Send CMD38 to set start erasing
    -------------------------------------------------------------------------*/
    mPeriph->ARG = 0u;
    mPeriph->CMD = CmdRegCfg[ CMD_ERASE ];

    return getCmdResp1( CMD_ERASE, SDIO_MAX_ERASE_TIMEOUT_MS );
  }


  ErrorType Driver::cmdEraseEndAdd( const uint32_t address )
  {
    /*-------------------------------------------------------------------------
    Send CMD33 to set the end address
    -------------------------------------------------------------------------*/
    mPeriph->ARG = address;
    mPeriph->CMD = CmdRegCfg[ CMD_SD_ERASE_GRP_END ];

    return getCmdResp1( CMD_SD_ERASE_GRP_END, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdEraseStartAdd( const uint32_t address )
  {
    /*-------------------------------------------------------------------------
    Send CMD32 to set the start address
    -------------------------------------------------------------------------*/
    mPeriph->ARG = address;
    mPeriph->CMD = CmdRegCfg[ CMD_SD_ERASE_GRP_START ];

    return getCmdResp1( CMD_SD_ERASE_GRP_START, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdGoIdleState()
  {
    /*-------------------------------------------------------------------------
    Send CMD0 to reset the SD card
    -------------------------------------------------------------------------*/
    mPeriph->ARG = 0;
    mPeriph->CMD = CmdRegCfg[ CMD_GO_IDLE_STATE ];

    /*-------------------------------------------------------------------------
    Wait for the command to complete. There is no response, so just wait for
    the command to indicate it has been sent.
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


  ErrorType Driver::cmdReadMultiBlock( const uint32_t address )
  {
    /*-------------------------------------------------------------------------
    Send CMD17 to read a single block
    -------------------------------------------------------------------------*/
    mPeriph->ARG = address;
    mPeriph->CMD = CmdRegCfg[ CMD_READ_MULT_BLOCK ];

    return getCmdResp1( CMD_READ_MULT_BLOCK, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdReadSingleBlock( const uint32_t address )
  {
    /*-------------------------------------------------------------------------
    Send CMD17 to read a single block
    -------------------------------------------------------------------------*/
    mPeriph->ARG = address;
    mPeriph->CMD = CmdRegCfg[ CMD_READ_SINGLE_BLOCK ];

    return getCmdResp1( CMD_READ_SINGLE_BLOCK, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdOperCond()
  {
    /*-------------------------------------------------------------------------
    Send CMD8 to verify SD card interface operating condition

    Operating Conditions Argument: Table 6-18
      - [31:12]: Reserved (shall be set to '0')
      -  [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
      -   [7:0]: Check Pattern (recommended 0xAA)
    -------------------------------------------------------------------------*/
    mPeriph->ARG = 0x000001AAu;
    mPeriph->CMD = CmdRegCfg[ CMD_HS_SEND_EXT_CSD ];

    return getCmdResp7();
  }


  ErrorType Driver::cmdSendSCR()
  {
    /*-------------------------------------------------------------------------
    Send CMD51 to read the SD Configuration Register (SCR)
    -------------------------------------------------------------------------*/
    mPeriph->ARG = 0;
    mPeriph->CMD = CmdRegCfg[ CMD_SD_APP_SEND_SCR ];

    return getCmdResp1( CMD_SD_APP_SEND_SCR, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdSendStatus( const uint16_t rca )
  {
    /*-------------------------------------------------------------------------
    Send CMD13 to get the SD card status
    -------------------------------------------------------------------------*/
    mPeriph->ARG = static_cast<uint32_t>( rca ) << 16u;
    mPeriph->CMD = CmdRegCfg[ CMD_SEND_STATUS ];

    return getCmdResp1( CMD_SEND_STATUS, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdSetRelAdd( uint16_t *const pRCA )
  {
    /*-------------------------------------------------------------------------
    Send CMD3 to get the SD card RCA
    -------------------------------------------------------------------------*/
    mPeriph->ARG = 0;
    mPeriph->CMD = CmdRegCfg[ CMD_SET_REL_ADDR ];

    const ErrorType err = getCmdResp6( CMD_SET_REL_ADDR, pRCA );
    if( err == ERROR_NONE )
    {
      mRCA = *pRCA;
    }

    return err;
  }


  ErrorType Driver::cmdStatusRegister()
  {
    /*-------------------------------------------------------------------------
    Send CMD13 to get the SD card status. Assumes CMD55 has already been sent
    with the card addressed.
    -------------------------------------------------------------------------*/
    mPeriph->ARG = 0u;
    mPeriph->CMD = CmdRegCfg[ CMD_SEND_STATUS ];

    return getCmdResp1( CMD_SEND_STATUS, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdSelDesel( const uint16_t rca )
  {
    /*-------------------------------------------------------------------------
    Send CMD7 to select the card
    -------------------------------------------------------------------------*/
    mPeriph->ARG = static_cast<uint32_t>( rca ) << 16u;
    mPeriph->CMD = CmdRegCfg[ CMD_SEL_DESEL_CARD ];

    return getCmdResp1( CMD_SEL_DESEL_CARD, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdSendCID()
  {
    /*-------------------------------------------------------------------------
    Send CMD2 to request the CID register
    -------------------------------------------------------------------------*/
    mPeriph->ARG = 0;
    mPeriph->CMD = CmdRegCfg[ CMD_ALL_SEND_CID ];

    return getCmdResp2();
  }


  ErrorType Driver::cmdSendCSD( const uint16_t rca )
  {
    /*-------------------------------------------------------------------------
    Send CMD9 to select/deselect the card
    -------------------------------------------------------------------------*/
    mPeriph->ARG = static_cast<uint32_t>( rca ) << 16u;
    mPeriph->CMD = CmdRegCfg[ CMD_SEND_CSD ];

    return getCmdResp2();
  }


  ErrorType Driver::cmdStopTransfer()
  {
    /*-------------------------------------------------------------------------
    Send CMD12 to stop a transmission
    -------------------------------------------------------------------------*/
    mPeriph->ARG = 0u;
    mPeriph->CMD = CmdRegCfg[ CMD_STOP_TRANSMISSION ];

    return getCmdResp1( CMD_STOP_TRANSMISSION, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdWriteMultiBlock( const uint32_t address )
  {
    /*-------------------------------------------------------------------------
    Send CMD25 to write a single block
    -------------------------------------------------------------------------*/
    mPeriph->ARG = address;
    mPeriph->CMD = CmdRegCfg[ CMD_WRITE_MULT_BLOCK ];

    return getCmdResp1( CMD_WRITE_MULT_BLOCK, SDIO_CMD_TIMEOUT_MS );
  }


  ErrorType Driver::cmdWriteSingleBlock( const uint32_t address )
  {
    /*-------------------------------------------------------------------------
    Send CMD24 to write a single block
    -------------------------------------------------------------------------*/
    mPeriph->ARG = address;
    mPeriph->CMD = CmdRegCfg[ CMD_WRITE_SINGLE_BLOCK ];

    return getCmdResp1( CMD_WRITE_SINGLE_BLOCK, SDIO_CMD_TIMEOUT_MS );
  }


  /*---------------------------------------------------------------------------
  Response Management
  ---------------------------------------------------------------------------*/
  ErrorType Driver::getCmdResp1( const uint8_t command, const uint32_t timeout ) const
  {
    using namespace Chimera::SDIO;

    /*-------------------------------------------------------------------------
    Do timeout on flag updates
    -------------------------------------------------------------------------*/
    if ( auto error = wait_cpsm_send_finish( mPeriph, timeout ); error != ERROR_NONE )
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

    /*-------------------------------------------------------------------------
    Exit in a known state by clearing all static flags
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );

    /*-------------------------------------------------------------------------
    Check response received is of desired command
    -------------------------------------------------------------------------*/
    if ( CMDRESP::get( mPeriph ) != command )
    {
      return ERROR_CMD_CRC_FAIL;
    }

    /*-------------------------------------------------------------------------
    Parse the response
    -------------------------------------------------------------------------*/
    auto r1 = cpsmGetRespX( ResponseMailbox::RESPONSE_1 );

    if ( ( r1 & OCR_ERRORBITS ) == 0 )
    {
      return ERROR_NONE;
    }
    else if ( ( r1 & OCR_ADDR_OUT_OF_RANGE ) == OCR_ADDR_OUT_OF_RANGE )
    {
      return ERROR_ADDR_OUT_OF_RANGE;
    }
    else if ( ( r1 & OCR_ADDR_MISALIGNED ) == OCR_ADDR_MISALIGNED )
    {
      return ERROR_ADDR_MISALIGNED;
    }
    else if ( ( r1 & OCR_BLOCK_LEN_ERR ) == OCR_BLOCK_LEN_ERR )
    {
      return ERROR_BLOCK_LEN_ERR;
    }
    else if ( ( r1 & OCR_ERASE_SEQ_ERR ) == OCR_ERASE_SEQ_ERR )
    {
      return ERROR_ERASE_SEQ_ERR;
    }
    else if ( ( r1 & OCR_BAD_ERASE_PARAM ) == OCR_BAD_ERASE_PARAM )
    {
      return ERROR_BAD_ERASE_PARAM;
    }
    else if ( ( r1 & OCR_WRITE_PROT_VIOLATION ) == OCR_WRITE_PROT_VIOLATION )
    {
      return ERROR_WRITE_PROT_VIOLATION;
    }
    else if ( ( r1 & OCR_LOCK_UNLOCK_FAILED ) == OCR_LOCK_UNLOCK_FAILED )
    {
      return ERROR_LOCK_UNLOCK_FAILED;
    }
    else if ( ( r1 & OCR_COM_CRC_FAILED ) == OCR_COM_CRC_FAILED )
    {
      return ERROR_COM_CRC_FAILED;
    }
    else if ( ( r1 & OCR_ILLEGAL_CMD ) == OCR_ILLEGAL_CMD )
    {
      return ERROR_ILLEGAL_CMD;
    }
    else if ( ( r1 & OCR_CARD_ECC_FAILED ) == OCR_CARD_ECC_FAILED )
    {
      return ERROR_CARD_ECC_FAILED;
    }
    else if ( ( r1 & OCR_CC_ERROR ) == OCR_CC_ERROR )
    {
      return ERROR_CC_ERR;
    }
    else if ( ( r1 & OCR_STREAM_READ_UNDERRUN ) == OCR_STREAM_READ_UNDERRUN )
    {
      return ERROR_STREAM_READ_UNDERRUN;
    }
    else if ( ( r1 & OCR_STREAM_WRITE_OVERRUN ) == OCR_STREAM_WRITE_OVERRUN )
    {
      return ERROR_STREAM_WRITE_OVERRUN;
    }
    else if ( ( r1 & OCR_CID_CSD_OVERWRITE ) == OCR_CID_CSD_OVERWRITE )
    {
      return ERROR_CID_CSD_OVERWRITE;
    }
    else if ( ( r1 & OCR_WP_ERASE_SKIP ) == OCR_WP_ERASE_SKIP )
    {
      return ERROR_WP_ERASE_SKIP;
    }
    else if ( ( r1 & OCR_CARD_ECC_DISABLED ) == OCR_CARD_ECC_DISABLED )
    {
      return ERROR_CARD_ECC_DISABLED;
    }
    else if ( ( r1 & OCR_ERASE_RESET ) == OCR_ERASE_RESET )
    {
      return ERROR_ERASE_RESET;
    }
    else if ( ( r1 & OCR_AKE_SEQ_ERROR ) == OCR_AKE_SEQ_ERROR )
    {
      return ERROR_AKE_SEQ_ERR;
    }
    else
    {
      return ERROR_GENERAL_UNKNOWN_ERR;
    }
  }


  ErrorType Driver::getCmdResp2() const
  {
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

    /*-------------------------------------------------------------------------
    Exit in a known state by clearing all static flags
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );
    return ERROR_NONE;
  }


  ErrorType Driver::getCmdResp3() const
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

    /*-------------------------------------------------------------------------
    Exit in a known state by clearing all static flags
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );
    return ERROR_NONE;
  }


  ErrorType Driver::getCmdResp6( const uint8_t command, uint16_t *const pRCA ) const
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
    /* else nothing to do */

    /*-------------------------------------------------------------------------
    Check response received is of desired command
    -------------------------------------------------------------------------*/
    if ( CMDRESP::get( mPeriph ) != command )
    {
      return ERROR_CMD_CRC_FAIL;
    }

    /*-------------------------------------------------------------------------
    Exit in a known state by clearing all static flags
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );

    /*-------------------------------------------------------------------------
    Get and parse the response
    -------------------------------------------------------------------------*/
    const auto r1 = cpsmGetRespX( ResponseMailbox::RESPONSE_1 );

    if ( ( r1 & ( R6_GENERAL_UNKNOWN_ERROR | R6_ILLEGAL_CMD | R6_COM_CRC_FAILED ) ) == 0 )
    {
      *pRCA = ( uint16_t )( r1 >> 16 );
      return ERROR_NONE;
    }
    else if ( ( r1 & R6_ILLEGAL_CMD ) == R6_ILLEGAL_CMD )
    {
      return ERROR_ILLEGAL_CMD;
    }
    else if ( ( r1 & R6_COM_CRC_FAILED ) == R6_COM_CRC_FAILED )
    {
      return ERROR_COM_CRC_FAILED;
    }
    else
    {
      return ERROR_GENERAL_UNKNOWN_ERR;
    }
  }


  ErrorType Driver::getCmdResp7() const
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


  /*---------------------------------------------------------------------------
  Data Management
  ---------------------------------------------------------------------------*/
  ErrorType Driver::getStreamControlRegister( const uint16_t rca, uint32_t *const pSCR )
  {
    constexpr uint32_t SDMMC_0TO7BITS   = 0x000000FFU;
    constexpr uint32_t SDMMC_8TO15BITS  = 0x0000FF00U;
    constexpr uint32_t SDMMC_16TO23BITS = 0x00FF0000U;
    constexpr uint32_t SDMMC_24TO31BITS = 0xFF000000U;

    ErrorType  error         = ErrorType::ERROR_NONE;
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
    if ( error = cmdAppCommand( rca ); error != ERROR_NONE )
    {
      goto exit_func;
    }

    /* Configure the SD DPSM (Data Path State Machine) */
    DATATIME::set( mPeriph, 5000 );
    DATALENGTH::set( mPeriph, 8u );
    DCTRL_ALL::set( mPeriph, DCTRL_DATABLOCK_SIZE_8B | DCTRL_DTDIR | DCTRL_DTEN );

    /* Send ACMD51 SD_APP_SEND_SCR */
    if ( error = cmdSendSCR(); error != ERROR_NONE )
    {
      goto exit_func;
    }

    while ( ( STA_ALL::get( mPeriph ) & ( STA_RXOVERR | STA_DCRCFAIL | STA_DTIMEOUT ) ) == 0 )
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
        error = ErrorType::ERROR_TIMEOUT;
        goto exit_func;
      }
    }

    if ( DTIMEOUT::get( mPeriph ) == STA_DTIMEOUT )
    {
      DTIMEOUTC::set( mPeriph, STA_DTIMEOUT );
      error = ErrorType::ERROR_DATA_TIMEOUT;
    }
    else if ( DCRCFAIL::get( mPeriph ) == STA_DCRCFAIL )
    {
      DCRCFAILC::set( mPeriph, STA_DCRCFAIL );
      error = ErrorType::ERROR_DATA_CRC_FAIL;
    }
    else if ( RXOVERR::get( mPeriph ) == STA_RXOVERR )
    {
      RXOVERRC::set( mPeriph, STA_RXOVERR );
      error = ErrorType::ERROR_RX_OVERRUN;
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

      error = ErrorType::ERROR_NONE;
    }

  exit_func:
    DCTRL_ALL::set( mPeriph, 0u );
    return error;
  }


  ErrorType Driver::getSDStatus( const uint16_t rca, uint32_t *const pSDstatus )
  {
    ErrorType error;
    uint32_t tickstart = Chimera::millis();
    uint32_t count;
    uint32_t *pData = pSDstatus;

    /*-------------------------------------------------------------------------
    Set the block size of the transfer
    -------------------------------------------------------------------------*/
    if ( error = cmdBlockLength( 64U ); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Send CMD55 to indicate the next command is an application specific command
    -------------------------------------------------------------------------*/
    if ( error = cmdAppCommand( rca ); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Set the DPSM configuration register
    -------------------------------------------------------------------------*/
    DATATIME::set( mPeriph, 5000 );
    DATALENGTH::set( mPeriph, 64u );
    DCTRL_ALL::set( mPeriph, DCTRL_DATABLOCK_SIZE_64B | DCTRL_DTDIR | DCTRL_DTEN );

    /*-------------------------------------------------------------------------
    Send ACMD13 to get the SD card status
    -------------------------------------------------------------------------*/
    if( error = cmdStatusRegister(); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Retrieve data from the FIFO in 8-byte chunks
    -------------------------------------------------------------------------*/
    uint32_t status = mPeriph->STA;
    while ( 0 == ( status & ( STA_RXOVERR | STA_DCRCFAIL | STA_DTIMEOUT | STA_DBCKEND ) ) )
    {
      if ( RXFIFOHF::get( mPeriph ) )
      {
          for ( count = 0U; count < 8U; count++ )
          {
            *pData = mPeriph->FIFO;
            pData++;
          }
      }

      if ( ( Chimera::millis() - tickstart ) >= SDIO_DATA_TIMEOUT_MS )
      {
          return ERROR_TIMEOUT;
      }
    }

    /*-------------------------------------------------------------------------
    Handle error codes
    -------------------------------------------------------------------------*/
    if ( DTIMEOUT::get( mPeriph ) == STA_DTIMEOUT )
    {
      DTIMEOUTC::set( mPeriph, ICR_DTIMEOUTC );
      return ERROR_DATA_TIMEOUT;
    }
    else if ( DCRCFAIL::get( mPeriph ) == STA_DCRCFAIL )
    {
      DCRCFAILC::set( mPeriph, ICR_DCRCFAILC );
      return ERROR_DATA_CRC_FAIL;
    }
    else if ( RXOVERR::get( mPeriph ) == STA_RXOVERR )
    {
      RXOVERRC::set( mPeriph, ICR_RXOVERRC );
      return ERROR_RX_OVERRUN;
    }

    /*-------------------------------------------------------------------------
    Pull all remaining data out of the FIFO
    -------------------------------------------------------------------------*/
    while ( RXDAVL::get( mPeriph ) )
    {
      *pData = mPeriph->FIFO;
      pData++;

      if ( ( Chimera::millis() - tickstart ) >= SDIO_DATA_TIMEOUT_MS )
      {
      return ERROR_TIMEOUT;
      }
    }

    /*-------------------------------------------------------------------------
    Exit in a known state by clearing all static flags
    -------------------------------------------------------------------------*/
    ICR_ALL::set( mPeriph, STATIC_CMD_FLAGS );

    return ERROR_NONE;
  }


  ErrorType Driver::getCardStatus( const uint16_t rca, uint32_t *const pCardStatus )
  {
    /*-------------------------------------------------------------------------
    Send CMD13 to get the card status
    -------------------------------------------------------------------------*/
    if ( auto error = cmdSendStatus( rca ); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Get the card status response
    -------------------------------------------------------------------------*/
    RT_DBG_ASSERT( pCardStatus );
    *pCardStatus = cpsmGetRespX( ResponseMailbox::RESPONSE_1 );
    return ERROR_NONE;
  }


  ErrorType Driver::asyncReadBlock( const uint32_t address, const uint32_t count, void *const buffer )
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Event;
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Pre-compute the number of bytes to transfer
    -------------------------------------------------------------------------*/
    const uint32_t bytes_to_transfer = count * SDIO_BLOCK_SIZE;

    /*-------------------------------------------------------------------------
    Fail early if the card is not ready
    -------------------------------------------------------------------------*/
    if( !waitUntilReady( 250u ) )
    {
      return ErrorType::ERROR_BUSY;
    }

    /*-------------------------------------------------------------------------
    Set the block size of the transfer
    -------------------------------------------------------------------------*/
    ErrorType error = ErrorType::ERROR_NONE;
    if ( error = cmdBlockLength( 512u ); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Make the request to read from the card
    -------------------------------------------------------------------------*/
    if ( count == 1 )
    {
      error = cmdReadSingleBlock( address );
    }
    else
    {
      error = cmdReadMultiBlock( address );
    }

    if ( error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Configure the DMA transfer, which will be started by the DPSM
    -------------------------------------------------------------------------*/
    PipeTransfer cfg;
    cfg.pipe = mRXDMAPipeId;
    cfg.size = bytes_to_transfer / sizeof( uint32_t );
    cfg.addr = reinterpret_cast<std::uintptr_t>( buffer );

    s_ctrl_blk[ mResourceIndex ].txfrId = Chimera::DMA::transfer( cfg );
    RT_DBG_ASSERT( s_ctrl_blk[ mResourceIndex ].txfrId != Chimera::DMA::INVALID_REQUEST );
    RT_DBG_ASSERT( cfg.addr % sizeof( uint32_t ) == 0 );

    /*-------------------------------------------------------------------------
    Configure interrupts for error handling
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    ICR_ALL::set( mPeriph, RX_ISR_CLEAR_FLAGS );

    INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );

    MASK_ALL::set( mPeriph, RX_ISR_MASK_FLAGS );

    /*-------------------------------------------------------------------------
    Configure the data path state machine. Once this starts, the DMA transfer
    will either complete or the SDIO peripheral will error out.
    -------------------------------------------------------------------------*/
    CLKEN::set( mPeriph, CLKCR_CLKEN );
    DATATIME::set( mPeriph, 5000 );
    DATALENGTH::set( mPeriph, bytes_to_transfer );
    DCTRL_ALL::set( mPeriph, DCTRL_DTEN | DCTRL_DTDIR | DCTRL_DMAEN | DCTRL_DATABLOCK_SIZE_512B );

    /*-------------------------------------------------------------------------
    Wait for the transfer to complete
    -------------------------------------------------------------------------*/
    auto wait_error = await( Trigger::TRIGGER_READ_COMPLETE, ( 5u * TIMEOUT_1MS ) );

    /*-------------------------------------------------------------------------
    Finalize the transfer
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Chimera::DMA::abort( s_ctrl_blk[ mResourceIndex ].txfrId );
    cmdStopTransfer();

    /*-------------------------------------------------------------------------
    Hand back an appropriate error code
    -------------------------------------------------------------------------*/
    if ( wait_error == Chimera::Status::OK )
    {
      return s_ctrl_blk[ mResourceIndex ].txfrError;
    }
    else
    {
      return ErrorType::ERROR_TIMEOUT;
    }
  }


  ErrorType Driver::asyncWriteBlock( const uint32_t address, const uint32_t count, const void *const buffer )
  {
    using namespace Chimera::DMA;
    using namespace Chimera::Event;
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Pre-compute the number of bytes to transfer
    -------------------------------------------------------------------------*/
    const uint32_t bytes_to_transfer = count * SDIO_BLOCK_SIZE;

    /*-------------------------------------------------------------------------
    Fail early if the card is not ready
    -------------------------------------------------------------------------*/
    if( !waitUntilReady( 250u ) )
    {
      return ErrorType::ERROR_BUSY;
    }

    /*-------------------------------------------------------------------------
    Set the block size of the transfer
    -------------------------------------------------------------------------*/
    ErrorType error = ErrorType::ERROR_NONE;
    if ( error = cmdBlockLength( 512u ); error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Request a write to the card
    -------------------------------------------------------------------------*/
    if ( count == 1 )
    {
      error = cmdWriteSingleBlock( address );
    }
    else
    {
      error = cmdWriteMultiBlock( address );
    }

    if ( error != ERROR_NONE )
    {
      return error;
    }

    /*-------------------------------------------------------------------------
    Configure the DMA transfer now that we know the card is ready
    -------------------------------------------------------------------------*/
    PipeTransfer cfg;
    cfg.pipe = mTXDMAPipeId;
    cfg.size = bytes_to_transfer / sizeof( uint32_t );
    cfg.addr = reinterpret_cast<std::uintptr_t>( buffer );

    s_ctrl_blk[ mResourceIndex ].txfrId = Chimera::DMA::transfer( cfg );
    RT_DBG_ASSERT( s_ctrl_blk[ mResourceIndex ].txfrId != Chimera::DMA::INVALID_REQUEST );
    RT_DBG_ASSERT( cfg.addr % sizeof( uint32_t ) == 0 );

    /*-------------------------------------------------------------------------
    Configure interrupts for error handling
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    ICR_ALL::set( mPeriph, TX_ISR_CLEAR_FLAGS );

    INT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ] );
    INT::enableIRQ( Resource::IRQSignals[ mResourceIndex ] );

    MASK_ALL::set( mPeriph, TX_ISR_MASK_FLAGS );

    /*-------------------------------------------------------------------------
    Configure the data path state machine. Once this starts, the DMA transfer
    will either complete or the SDIO peripheral will error out.
    -------------------------------------------------------------------------*/
    CLKEN::set( mPeriph, CLKCR_CLKEN );
    DATATIME::set( mPeriph, 5000 );
    DATALENGTH::set( mPeriph, bytes_to_transfer );
    DCTRL_ALL::set( mPeriph, DCTRL_DTEN | DCTRL_DMAEN | DCTRL_DATABLOCK_SIZE_512B );

    /*-------------------------------------------------------------------------
    Wait for the transfer to complete
    -------------------------------------------------------------------------*/
    auto wait_error = await( Trigger::TRIGGER_WRITE_COMPLETE, ( 5u * TIMEOUT_1MS ) );

    /*-------------------------------------------------------------------------
    Finalize the transfer
    -------------------------------------------------------------------------*/
    INT::disableIRQ( Resource::IRQSignals[ mResourceIndex ] );
    Chimera::DMA::abort( s_ctrl_blk[ mResourceIndex ].txfrId );
    cmdStopTransfer();

    /*-------------------------------------------------------------------------
    Hand back an appropriate error code
    -------------------------------------------------------------------------*/
    if ( wait_error == Chimera::Status::OK )
    {
      /*-----------------------------------------------------------------------
      Don't return until the card is ready for the next transfer
      -----------------------------------------------------------------------*/
      if( !waitUntilReady( 250u ) )
      {
        return ErrorType::ERROR_BUSY;
      }

      return s_ctrl_blk[ mResourceIndex ].txfrError;
    }
    else
    {
      return ErrorType::ERROR_TIMEOUT;
    }
  }


  void Driver::IRQHandler()
  {
    using namespace Chimera::Event;

    /*-------------------------------------------------------------------------
    Read the status and mask registers
    -------------------------------------------------------------------------*/
    const uint32_t status = mPeriph->STA;
    const uint32_t mask   = mPeriph->MASK;
    const uint32_t ctrl   = mPeriph->DCTRL;

    /*-------------------------------------------------------------------------
    Handle TX errors and completion events
    -------------------------------------------------------------------------*/
    /* clang-format off */
    if ( ( ( ctrl & DCTRL_DTDIR ) == TRANSFER_DIR_TO_CARD ) &&
         ( status & TX_ISR_STATUS_FLAGS                   ) &&
         (   mask & TX_ISR_MASK_FLAGS                     ) )
    { /* clang-format on */
      s_ctrl_blk[ mResourceIndex ].txfrStatus = status;
      s_ctrl_blk[ mResourceIndex ].txfrError  = ErrorType::ERROR_GENERAL_UNKNOWN_ERR;

      /*-----------------------------------------------------------------------
      Disable and ACK the interrupts. Reset DMA controller.
      -----------------------------------------------------------------------*/
      MASK_ALL::clear( mPeriph, TX_ISR_MASK_FLAGS );
      ICR_ALL::set( mPeriph, TX_ISR_CLEAR_FLAGS );
      DCTRL_ALL::set( mPeriph, 0 );

      /*-----------------------------------------------------------------------
      Parse the reason for the interrupt
      -----------------------------------------------------------------------*/
      if ( ( status & STA_TXUNDERR ) && ( mask & MASK_TXUNDERRIE ) )
      {
        s_ctrl_blk[ mResourceIndex ].txfrError = ErrorType::ERROR_TX_UNDERRUN;
      }

      if ( ( status & STA_DTIMEOUT ) && ( mask & MASK_DTIMEOUTIE ) )
      {
        s_ctrl_blk[ mResourceIndex ].txfrError = ErrorType::ERROR_DATA_TIMEOUT;
      }

      if ( ( status & STA_DCRCFAIL ) && ( mask & MASK_DCRCFAILIE ) )
      {
        s_ctrl_blk[ mResourceIndex ].txfrError = ErrorType::ERROR_DATA_CRC_FAIL;
      }

      if ( ( ( status & STA_DBCKEND ) && ( mask & MASK_DBCKENDIE ) ) ||
           ( ( status & STA_DATAEND ) && ( mask & MASK_DATAENDIE ) ) )
      {
        s_ctrl_blk[ mResourceIndex ].txfrError = ErrorType::ERROR_NONE;
      }

      /*-------------------------------------------------------------------------
      Signal the transfer is complete
      -------------------------------------------------------------------------*/
      signalAIOFromISR( Trigger::TRIGGER_WRITE_COMPLETE );
      return;
    }

    /*-------------------------------------------------------------------------
    Handle RX errors and completion events
    -------------------------------------------------------------------------*/
    /* clang-format off */
    if ( ( ( ctrl & DCTRL_DTDIR ) == TRANSFER_DIR_TO_SDIO ) &&
         ( status & RX_ISR_STATUS_FLAGS                   ) &&
         (   mask & RX_ISR_MASK_FLAGS                     ) )
    { /* clang-format on */
      s_ctrl_blk[ mResourceIndex ].txfrStatus = status;
      s_ctrl_blk[ mResourceIndex ].txfrError  = ErrorType::ERROR_GENERAL_UNKNOWN_ERR;

      /*-----------------------------------------------------------------------
      Disable and ACK the interrupts. Reset DMA controller.
      -----------------------------------------------------------------------*/
      MASK_ALL::clear( mPeriph, RX_ISR_MASK_FLAGS );
      ICR_ALL::set( mPeriph, RX_ISR_CLEAR_FLAGS );
      DCTRL_ALL::set( mPeriph, 0 );

      /*-----------------------------------------------------------------------
      Parse the reason for the interrupt
      -----------------------------------------------------------------------*/
      if ( ( status & STA_RXOVERR ) && ( mask & MASK_RXOVERRIE ) )
      {
        s_ctrl_blk[ mResourceIndex ].txfrError = ErrorType::ERROR_RX_OVERRUN;
      }

      if ( ( status & STA_DTIMEOUT ) && ( mask & MASK_DTIMEOUTIE ) )
      {
        s_ctrl_blk[ mResourceIndex ].txfrError = ErrorType::ERROR_DATA_TIMEOUT;
      }

      if ( ( status & STA_DCRCFAIL ) && ( mask & MASK_DCRCFAILIE ) )
      {
        s_ctrl_blk[ mResourceIndex ].txfrError = ErrorType::ERROR_DATA_CRC_FAIL;
      }

      if ( ( ( status & STA_DBCKEND ) && ( mask & MASK_DBCKENDIE ) ) ||
           ( ( status & STA_DATAEND ) && ( mask & MASK_DATAENDIE ) ) )
      {
        s_ctrl_blk[ mResourceIndex ].txfrError = ErrorType::ERROR_NONE;
      }

      /*-------------------------------------------------------------------------
      Signal the transfer is complete
      -------------------------------------------------------------------------*/
      signalAIOFromISR( Trigger::TRIGGER_READ_COMPLETE );
      return;
    }
  }


  bool Driver::readyForNextTransfer()
  {
    using namespace Chimera::SDIO;

    /*-------------------------------------------------------------------------
    Look at the status register of the card for the "ready for data" bit
    -------------------------------------------------------------------------*/
    uint32_t cardStatus = 0;
    getCardStatus( mRCA, &cardStatus );

    return ( cardStatus & CMD13_READY_FOR_DATA ) == CMD13_READY_FOR_DATA;
  }


  bool Driver::waitUntilReady( const uint32_t timeout )
  {
    using namespace Chimera::Thread;

    const size_t start_time = Chimera::millis();
    const size_t end_time   = start_time + ( timeout * TIMEOUT_1MS );

    while ( !readyForNextTransfer() )
    {
      if ( Chimera::millis() >= end_time )
      {
        return false;
      }

      Chimera::delayMilliseconds( 5 );
    }

    return true;
  }
}    // namespace Thor::LLD::SDIO
#endif /* THOR_SDIO && TARGET_STM32F4 */
