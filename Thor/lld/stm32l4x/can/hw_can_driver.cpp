/********************************************************************************
 *  File Name:
 *    hw_can_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series CAN hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <limits>

/* Aurora Includes */
#include <Aurora/math>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/common>
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_prv_data.hpp>
#include <Thor/lld/stm32l4x/can/hw_can_prj.hpp>
#include <Thor/lld/stm32l4x/can/hw_can_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_CAN )

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Static Variables
  -------------------------------------------------------------------------------*/
  static Driver s_can_drivers[ NUM_CAN_PERIPHS ];

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();

    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_can_drivers, ARRAY_COUNT( s_can_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Driver_rPtr getDriver( const Chimera::CAN::Channel channel )
  {
    if ( auto idx = getResourceIndex( channel ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_can_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  void prv_reset( RegisterMap *const periph )
  {
    MCR_ALL::set( periph, MCR_Rst );
    MSR_ALL::set( periph, MSR_Rst );
    TSR_ALL::set( periph, TSR_Rst );
    RF0R_ALL::set( periph, RF0R_Rst );
    RF1R_ALL::set( periph, RF1R_Rst );
    IER_ALL::set( periph, IER_Rst );
    ESR_ALL::set( periph, ESR_Rst );
    BTR_ALL::set( periph, BTR_Rst );
  }


  void prv_enter_initialization_mode( RegisterMap *const periph )
  {
    /*-------------------------------------------------
    If we are already in sleep mode, the sleep bit must
    be cleared first. See RM0394 44.4.3.
    -------------------------------------------------*/
    if ( SLEEP::get( periph ) )
    {
      prv_exit_sleep_mode( periph );
    }

    /*-------------------------------------------------
    Ask to enter init mode
    -------------------------------------------------*/
    INRQ::set( periph, MCR_INRQ );
    while ( !INAK::get( periph ) )
    {
      continue;
    }
  }


  void prv_exit_initialization_mode( RegisterMap *const periph )
  {
    INRQ::clear( periph, MCR_INRQ );
    while ( INAK::get( periph ) )
    {
      continue;
    }
  }


  void prv_enter_normal_mode( RegisterMap *const periph )
  {
    /*-------------------------------------------------
    This command is the same
    -------------------------------------------------*/
    prv_exit_initialization_mode( periph );
  }


  void prv_enter_sleep_mode( RegisterMap *const periph )
  {
    SLEEP::set( periph, MCR_SLEEP );
    while ( !SLAK::get( periph ) )
    {
      continue;
    }
  }


  void prv_exit_sleep_mode( RegisterMap *const periph )
  {
    SLEEP::clear( periph, MCR_SLEEP );
    while ( SLAK::get( periph ) )
    {
      continue;
    }
  }


  size_t prv_set_baud_rate( RegisterMap *const periph, const Chimera::CAN::DriverConfig &cfg )
  {
    /*-------------------------------------------------
    Grab a few preliminary variables. These equations
    are derived from RM0394 Figure 488.
    -------------------------------------------------*/
    size_t pclk = Thor::LLD::RCC::getCoreClock()->getPeriphClock( Chimera::Peripheral::Type::PERIPH_CAN,
                                                                  reinterpret_cast<std::uintptr_t>( periph ) );

    float nominalBitTime = 1.0f / ( float )( cfg.HWInit.baudRate );
    float clkPeriod      = 1.0f / ( float )( pclk );
    float tq             = nominalBitTime / ( float )( cfg.HWInit.timeQuanta );
    float tsync          = tq;
    float est_tbs1       = ( cfg.HWInit.samplePointPercent * nominalBitTime ) - tsync;
    float est_tbs2       = nominalBitTime - tsync - est_tbs1;

    /*-------------------------------------------------
    Calculate configuration settings for BTR register
    -------------------------------------------------*/
    Reg32_t brp = static_cast<Reg32_t>( tq / clkPeriod ) - 1;
    Reg32_t ts1 = static_cast<Reg32_t>( est_tbs1 / tq ) - 1;
    Reg32_t ts2 = static_cast<Reg32_t>( est_tbs2 / tq ) - 1;

    /*-------------------------------------------------
    Apply the configuration values
    -------------------------------------------------*/
    BRP::set( periph, ( brp << BTR_BRP_Pos ) );
    TS1::set( periph, ( ts1 << BTR_TS1_Pos ) );
    TS2::set( periph, ( ts2 << BTR_TS2_Pos ) );

    /*-------------------------------------------------
    Calculate the actual configured baud rate
    -------------------------------------------------*/
    return prv_get_baud_rate( periph );
  }


  size_t prv_get_baud_rate( RegisterMap *const periph )
  {
    using namespace Thor::LLD::RCC;
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------
    Grab a few preliminary variables
    -------------------------------------------------*/
    Reg32_t brp = BRP::get( periph ) >> BTR_BRP_Pos;
    Reg32_t ts1 = TS1::get( periph ) >> BTR_TS1_Pos;
    Reg32_t ts2 = TS2::get( periph ) >> BTR_TS2_Pos;
    size_t pclk = getCoreClock()->getPeriphClock( Type::PERIPH_CAN, reinterpret_cast<std::uintptr_t>( periph ) );

    /*-------------------------------------------------
    Prevent div/0 in calculations below
    -------------------------------------------------*/
    if ( !pclk )
    {
      return std::numeric_limits<size_t>::max();
    }

    /*-------------------------------------------------
    Calculate the expected output baud rate. Taken from
    RM0394 Figure 488.
    -------------------------------------------------*/
    float tq             = ( 1.0f / ( float )pclk ) * ( float )( brp + 1 );
    float tbs1           = tq * ( float )( ts1 + 1 );
    float tbs2           = tq * ( float )( ts2 + 1 );
    float nominalBitTime = tq + tbs1 + tbs2;

    /*-------------------------------------------------
    Assume that a bit time corresponding to 10 MBit is
    invalid (not supported by standard). Also prevents
    a div/0 without having to do extremely precise
    floating point comparisons.
    -------------------------------------------------*/
    if ( nominalBitTime < 0.0000001f )
    {
      return std::numeric_limits<size_t>::max();
    }
    else
    {
      return static_cast<size_t>( 1.0f / nominalBitTime );
    }
  }


  bool prv_validate_frame( const Chimera::CAN::BasicFrame &frame )
  {
    using namespace Chimera::CAN;

    bool result = true;

    result |= ( frame.dataLength != 0 );
    result |= ( frame.idMode < IdentifierMode::NUM_OPTIONS );
    //result |= ( frame.id is valid? What makes a valid id? )

    return result;
  }

  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() :
      mPeriph( nullptr ), mResourceIndex( std::numeric_limits<size_t>::max() ), mTXPin( nullptr ), mRXPin( nullptr )
  {
  }


  Driver::~Driver()
  {
  }

  /*-------------------------------------------------------------------------------
  Configuration
  -------------------------------------------------------------------------------*/
  void Driver::attach( RegisterMap *const peripheral )
  {
    /*------------------------------------------------
    Get peripheral descriptor settings
    ------------------------------------------------*/
    mPeriph        = peripheral;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*------------------------------------------------
    Handle the ISR configuration
    ------------------------------------------------*/
    for ( auto handlerIdx = 0; handlerIdx < NUM_CAN_IRQ_HANDLERS; handlerIdx++ )
    {
      /*-------------------------------------------------
      Configure the NVIC with the desired settings
      -------------------------------------------------*/
      Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ handlerIdx ] );
      Thor::LLD::IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ handlerIdx ] );
      Thor::LLD::IT::setPriority( Resource::IRQSignals[ mResourceIndex ][ handlerIdx ],
                                  Thor::Interrupt::CAN_IT_PREEMPT_PRIORITY, 0u );

      /*-------------------------------------------------
      Reset the semaphores to their un-signaled state
      -------------------------------------------------*/
      mISREventSignal[ handlerIdx ].try_acquire();
    }
  }


  Chimera::Status_t Driver::configure( const Chimera::CAN::DriverConfig &cfg )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Reset the driver registers to default values then
    configure hardware for initialization.
    -------------------------------------------------*/
    prv_reset( mPeriph );
    prv_enter_initialization_mode( mPeriph );

    /*-------------------------------------------------
    Initialize the GPIO drivers
    -------------------------------------------------*/
    if ( !mTXPin )
    {
      mTXPin = Chimera::GPIO::getDriver( cfg.TXInit.port, cfg.TXInit.pin );
    }

    if ( !mRXPin )
    {
      mRXPin = Chimera::GPIO::getDriver( cfg.RXInit.port, cfg.RXInit.pin );
    }

    mTXPin->init( cfg.TXInit );
    mRXPin->init( cfg.RXInit );

    /*-------------------------------------------------
    Set up Bit Timing
    -------------------------------------------------*/
    float actualBaud  = static_cast<float>( prv_set_baud_rate( mPeriph, cfg ) );
    float desiredBaud = static_cast<float>( cfg.HWInit.baudRate );

    if ( cfg.HWInit.maxBaudError > std::abs( Aurora::Math::percentError( actualBaud, desiredBaud ) ) )
    {
      return Chimera::Status::FAILED_INIT;
    }

    /*-------------------------------------------------
    Request to leave init mode
    -------------------------------------------------*/
    prv_exit_initialization_mode( mPeriph );
  }


  Chimera::Status_t Driver::applyFilter( const Chimera::CAN::Filter &filter )
  {
    // Must be done in init mode


    /*-------------------------------------------------
    Set up filter scaling and mode. These bits are shared
    across a wide nubmer of
    -------------------------------------------------*/
    // switch( cfg.HWInit.filterMode )
    // {
    //   case Chimera::CAN::FilterMode::ID_LIST:

    //     break;

    //   case Chimera::CAN::FilterMode::MASK:

    //     break;

    //   default:
    //     return Chimera::Status::FAILED_INIT;
    //     break;
    // }

    // switch( cfg.HWInit.filterWidth )
    // {
    //   case Chimera::CAN::FilterWidth::WIDTH_16BIT:

    //     break;

    //   case Chimera::CAN::FilterWidth::WIDTH_32BIT:

    //     break;

    //   default:
    //     return Chimera::Status::FAILED_INIT;
    //     break;
    // }
  }


  Chimera::Status_t Driver::enableISRSignal( const Chimera::CAN::InterruptType signal )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Enable an ISR event based on RM0394 Fig. 490. Proper
    functionality assumes that the NVIC controller has
    been correctly initialized.
    -------------------------------------------------*/
    switch ( signal )
    {
      /*-------------------------------------------------
      Transmit Interrupts
      -------------------------------------------------*/
      case InterruptType::TRANSMIT_MAILBOX_EMPTY:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_TX_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_TX_ISR_SIGNAL_INDEX ] );
        TMEIE::set( mPeriph, IER_TMEIE );
        break;

      /*-------------------------------------------------
      FIFO Interrupts: Operate on both FIFOs at once
      -------------------------------------------------*/
      case InterruptType::RECEIVE_FIFO_NEW_MESSAGE:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        FMPIE0::set( mPeriph, IER_FMPIE0 );
        FMPIE1::set( mPeriph, IER_FMPIE1 );
        break;

      case InterruptType::RECEIVE_FIFO_FULL:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        FFIE0::set( mPeriph, IER_FFIE0 );
        FFIE1::set( mPeriph, IER_FFIE1 );
        break;

      case InterruptType::RECEIVE_FIFO_OVERRUN:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        FOVIE0::set( mPeriph, IER_FOVIE0 );
        FOVIE1::set( mPeriph, IER_FOVIE1 );
        break;

      /*-------------------------------------------------
      Status Change Interrupts
      -------------------------------------------------*/
      case InterruptType::SLEEP_EVENT:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_STS_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_STS_ISR_SIGNAL_INDEX ] );
        SLKIE::set( mPeriph, IER_SLKIE );
        break;

      case InterruptType::WAKEUP_EVENT:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_STS_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_STS_ISR_SIGNAL_INDEX ] );
        WKUIE::set( mPeriph, IER_WKUIE );
        break;

      /*-------------------------------------------------
      Error Interrupts: Error signals are additionally
      masked by ERRIE, so ensure it is on.
      -------------------------------------------------*/
      case InterruptType::ERROR_CODE_EVENT:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        ERRIE::set( mPeriph, IER_ERRIE );
        LECIE::set( mPeriph, IER_LECIE );
        break;

      case InterruptType::ERROR_BUS_OFF_EVENT:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        ERRIE::set( mPeriph, IER_ERRIE );
        BOFIE::set( mPeriph, IER_BOFIE );
        break;

      case InterruptType::ERROR_PASSIVE_EVENT:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        ERRIE::set( mPeriph, IER_ERRIE );
        EPVIE::set( mPeriph, IER_EPVIE );
        break;

      case InterruptType::ERROR_WARNING_EVENT:
        IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        ERRIE::set( mPeriph, IER_ERRIE );
        EWGIE::set( mPeriph, IER_EWGIE );
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
        break;
    };
  }


  void Driver::disableISRSignal( const Chimera::CAN::InterruptType signal )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Disable an ISR event based on RM0394 Fig. 490
    -------------------------------------------------*/
    switch ( signal )
    {
      /*-------------------------------------------------
      Transmit Interrupts
      -------------------------------------------------*/
      case InterruptType::TRANSMIT_MAILBOX_EMPTY:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_TX_ISR_SIGNAL_INDEX ] );
        TMEIE::clear( mPeriph, IER_TMEIE );
        break;

      /*-------------------------------------------------
      FIFO Interrupts
      -------------------------------------------------*/
      case InterruptType::RECEIVE_FIFO_NEW_MESSAGE:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        FMPIE0::clear( mPeriph, IER_FMPIE0 );
        FMPIE1::clear( mPeriph, IER_FMPIE1 );
        break;

      case InterruptType::RECEIVE_FIFO_FULL:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        FFIE0::clear( mPeriph, IER_FFIE0 );
        FFIE1::clear( mPeriph, IER_FFIE1 );
        break;

      case InterruptType::RECEIVE_FIFO_OVERRUN:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_RX_ISR_SIGNAL_INDEX ] );
        FOVIE0::clear( mPeriph, IER_FOVIE0 );
        FOVIE1::clear( mPeriph, IER_FOVIE1 );
        break;

      /*-------------------------------------------------
      Status Change Interrupts
      -------------------------------------------------*/
      case InterruptType::SLEEP_EVENT:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_STS_ISR_SIGNAL_INDEX ] );
        SLKIE::clear( mPeriph, IER_SLKIE );
        break;

      case InterruptType::WAKEUP_EVENT:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_STS_ISR_SIGNAL_INDEX ] );
        WKUIE::clear( mPeriph, IER_WKUIE );
        break;

      /*-------------------------------------------------
      Error Interrupts
      -------------------------------------------------*/
      case InterruptType::ERROR_CODE_EVENT:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        LECIE::clear( mPeriph, IER_LECIE );
        break;

      case InterruptType::ERROR_BUS_OFF_EVENT:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        BOFIE::clear( mPeriph, IER_BOFIE );
        break;

      case InterruptType::ERROR_PASSIVE_EVENT:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        EPVIE::clear( mPeriph, IER_EPVIE );
        break;

      case InterruptType::ERROR_WARNING_EVENT:
        IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ CAN_ERR_ISR_SIGNAL_INDEX ] );
        EWGIE::clear( mPeriph, IER_EWGIE );
        break;

      default:
        // Do nothing, it's not supported.
        break;
    };
  }


  void Driver::enterDebugMode( const Chimera::CAN::DebugMode mode )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Request to configure the hardware
    -------------------------------------------------*/
    prv_enter_initialization_mode( mPeriph );

    /*-------------------------------------------------
    Appropriately configure the debug mode
    -------------------------------------------------*/
    switch ( mode )
    {
      case DebugMode::SILENT:
        SLIM::set( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::SILENT ) ] );
        LBKM::clear( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::LOOPBACK ) ] );
        break;

      case DebugMode::LOOPBACK:
        SLIM::clear( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::SILENT ) ] );
        LBKM::set( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::LOOPBACK ) ] );
        break;

      case DebugMode::LOOPBACK_AND_SILENT:
        SLIM::set( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::SILENT ) ] );
        LBKM::set( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::LOOPBACK ) ] );
        break;

      default:
        SLIM::clear( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::SILENT ) ] );
        LBKM::clear( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::LOOPBACK ) ] );
        break;
    };

    /*-------------------------------------------------
    Apply the debug configuration
    -------------------------------------------------*/
    prv_enter_normal_mode( mPeriph );
  }


  void Driver::exitDebugMode()
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Go to init mode, reconfigure, go to normal mode
    -------------------------------------------------*/
    prv_enter_initialization_mode( mPeriph );
    SLIM::clear( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::SILENT ) ] );
    LBKM::clear( mPeriph, ConfigMap::DebugMode[ static_cast<size_t>( DebugMode::LOOPBACK ) ] );
    prv_enter_normal_mode( mPeriph );
  }


  /*-------------------------------------------------------------------------------
  Transmit & Receive Operations
  -------------------------------------------------------------------------------*/
  bool Driver::txMailboxAvailable( Mailbox &which )
  {
    /*-------------------------------------------------
    Return the first mailbox that is not empty
    -------------------------------------------------*/
    if ( TME0::get( mPeriph ) )
    {
      which = Mailbox::TX_MAILBOX_1;
      return true;
    }
    else if ( TME1::get( mPeriph ) )
    {
      which = Mailbox::TX_MAILBOX_2;
      return true;
    }
    else if ( TME2::get( mPeriph ) )
    {
      which = Mailbox::TX_MAILBOX_3;
      return true;
    }
    else
    {
      which = Mailbox::UNKNOWN;
      return false;
    }
  }


  bool Driver::rxMailboxAvailable( Mailbox &which )
  {
    /*-------------------------------------------------
    Return the first FIFO that has a pending message
    -------------------------------------------------*/
    size_t fifo0PendingMessages = FMP0::get( mPeriph ) >> RF0R_FMP0_Pos;
    size_t fifo1PendingMessages = FMP1::get( mPeriph ) >> RF1R_FMP1_Pos;

    if ( fifo0PendingMessages )
    {
      which = Mailbox::RX_MAILBOX_1;
      return true;
    }
    else if ( fifo1PendingMessages )
    {
      which = Mailbox::RX_MAILBOX_2;
      return true;
    }
    else
    {
      which = Mailbox::UNKNOWN;
      return false;
    }
  }


  Chimera::Status_t Driver::send( const Mailbox which, const Chimera::CAN::BasicFrame &frame )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Can we even send this type of frame?
    -------------------------------------------------*/
    if( !prv_validate_frame( frame ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Enter a critical section to guarantee the mailbox
    will only be modified by this function.
    -------------------------------------------------*/
    enterCriticalSection();

    /*-------------------------------------------------
    Validate that the mailbox is actually free
    -------------------------------------------------*/
    volatile TxMailbox* box = nullptr;

    switch ( which )
    {
      case Mailbox::TX_MAILBOX_1:
        if ( !TME0::get( mPeriph ) )
        {
          exitCriticalSection();
          return Chimera::Status::NOT_READY;
        }

        box = &mPeriph->sTxMailBox[ RIDX_TX_MAILBOX_1 ];
        break;

      case Mailbox::TX_MAILBOX_2:
        if ( !TME1::get( mPeriph ) )
        {
          exitCriticalSection();
          return Chimera::Status::NOT_READY;
        }

        box = &mPeriph->sTxMailBox[ RIDX_TX_MAILBOX_2 ];
        break;

      case Mailbox::TX_MAILBOX_3:
        if ( !TME2::get( mPeriph ) )
        {
          exitCriticalSection();
          return Chimera::Status::NOT_READY;
        }

        box = &mPeriph->sTxMailBox[ RIDX_TX_MAILBOX_3 ];
        break;

      default:
        exitCriticalSection();
        return Chimera::Status::INVAL_FUNC_PARAM;
        break;
    };

    /*-------------------------------------------------
    Reset the mailbox
    -------------------------------------------------*/
    box->TIR  = 0;
    box->TDHR = 0;
    box->TDLR = 0;
    box->TDTR = 0;

    /*-------------------------------------------------
    Assign the STD/EXT ID
    -------------------------------------------------*/
    if( frame.idMode == IdentifierMode::STANDARD )
    {
      box->TIR |= ( frame.id & ID_MASK_11_BIT ) << TIxR_STID_Pos;
      box->TIR &= ~TIxR_IDE;
    }
    else // Extended
    {
      box->TIR |= ( ( frame.id & ID_MASK_29_BIT ) << TIxR_EXID_Pos );
      box->TIR |= TIxR_IDE;
    }

    /*-------------------------------------------------
    Assign the remote transmition type
    -------------------------------------------------*/
    if( frame.frameType == FrameType::DATA )
    {
      box->TIR &= ~TIxR_RTR;
    }
    else // Remote frame
    {
      box->TIR |= TIxR_RTR;
    }

    /*-------------------------------------------------
    Assign byte length of the transfer
    -------------------------------------------------*/
    box->TDTR |= ( ( frame.dataLength & TDT0R_DLC_Msk ) << TDT0R_DLC_Pos );

    /*-------------------------------------------------
    Assign the data payload
    -------------------------------------------------*/
    box->TDLR |= ( ( frame.data[ 0 ] & 0xFF ) << TDL0R_DATA0_Pos );
    box->TDLR |= ( ( frame.data[ 1 ] & 0xFF ) << TDL0R_DATA1_Pos );
    box->TDLR |= ( ( frame.data[ 2 ] & 0xFF ) << TDL0R_DATA2_Pos );
    box->TDLR |= ( ( frame.data[ 3 ] & 0xFF ) << TDL0R_DATA3_Pos );
    box->TDHR |= ( ( frame.data[ 4 ] & 0xFF ) << TDH0R_DATA4_Pos );
    box->TDHR |= ( ( frame.data[ 5 ] & 0xFF ) << TDH0R_DATA5_Pos );
    box->TDHR |= ( ( frame.data[ 6 ] & 0xFF ) << TDH0R_DATA6_Pos );
    box->TDHR |= ( ( frame.data[ 7 ] & 0xFF ) << TDH0R_DATA7_Pos );

    /*-------------------------------------------------
    Ensure interrupts are enabled for important events
    -------------------------------------------------*/
    enableISRSignal( InterruptType::TRANSMIT_MAILBOX_EMPTY );

    /*-------------------------------------------------
    Finally, move the mailbox to a "ready for tx" state
    -------------------------------------------------*/
    box->TIR |= TIxR_TXRQ;

    /*-------------------------------------------------
    Exit the critical section and return success.
    -------------------------------------------------*/
    exitCriticalSection();
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::receive( const Mailbox which, Chimera::CAN::BasicFrame &frame )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Enter a critical section to guarantee the mailbox
    will only be modified by this function.
    -------------------------------------------------*/
    enterCriticalSection();

    /*-------------------------------------------------
    Ensure the FIFO actually has data for us
    -------------------------------------------------*/
    volatile RxMailbox* box = nullptr;

    switch( which )
    {
      case Mailbox::RX_MAILBOX_1:
        if( !FMP0::get( mPeriph ) )
        {
          exitCriticalSection();
          return Chimera::Status::NOT_READY;
        }

        box = &mPeriph->sFIFOMailBox[ RIDX_RX_MAILBOX_1 ];
        break;

      case Mailbox::RX_MAILBOX_2:
        if( !FMP1::get( mPeriph ) )
        {
          exitCriticalSection();
          return Chimera::Status::NOT_READY;
        }

        box = &mPeriph->sFIFOMailBox[ RIDX_RX_MAILBOX_2 ];
        break;

      default:
        exitCriticalSection();
        frame.clear();
        return Chimera::Status::NOT_AVAILABLE;
        break;
    }

    /*-------------------------------------------------
    Makes sure there isn't strange data in the frame
    -------------------------------------------------*/
    frame.clear();

    /*-------------------------------------------------
    Read out the STD/EXT ID
    -------------------------------------------------*/
    if( !( box->RIR & RI0R_IDE ) )
    {
      frame.id = ( ( box->RIR & RI0R_STID_Msk ) >> RI0R_STID_Pos );
    }
    else // Extended
    {
      frame.id = ( ( box->RIR & RI0R_EXID_Msk ) >> RI0R_EXID_Pos );
    }

    /*-------------------------------------------------
    Read out the frame type
    -------------------------------------------------*/
    if( !( box->RIR & RI0R_RTR ))
    {
      frame.frameType = FrameType::DATA;
    }
    else // Remote
    {
      frame.frameType = FrameType::REMOTE;
    }

    /*-------------------------------------------------
    Read out the filter match idx and payload length
    -------------------------------------------------*/
    frame.filterIndex = ( ( box->RDTR & RDT0R_FMI_Msk ) >> RDT0R_FMI_Pos );
    frame.dataLength = ( ( box->RDTR & RDT0R_DLC_Msk ) >> RDT0R_DLC_Pos );

    /*-------------------------------------------------
    Read out the payload
    -------------------------------------------------*/
    const Reg32_t dataLo = box->RDLR;
    const Reg32_t dataHi = box->RDHR;

    frame.data[ 0 ] = ( ( dataLo & RDL0R_DATA0_Msk ) >> RDL0R_DATA0_Pos );
    frame.data[ 1 ] = ( ( dataLo & RDL0R_DATA1_Msk ) >> RDL0R_DATA1_Pos );
    frame.data[ 2 ] = ( ( dataLo & RDL0R_DATA2_Msk ) >> RDL0R_DATA2_Pos );
    frame.data[ 3 ] = ( ( dataLo & RDL0R_DATA3_Msk ) >> RDL0R_DATA3_Pos );
    frame.data[ 4 ] = ( ( dataHi & RDH0R_DATA4_Msk ) >> RDH0R_DATA4_Pos );
    frame.data[ 5 ] = ( ( dataHi & RDH0R_DATA5_Msk ) >> RDH0R_DATA5_Pos );
    frame.data[ 6 ] = ( ( dataHi & RDH0R_DATA6_Msk ) >> RDH0R_DATA6_Pos );
    frame.data[ 7 ] = ( ( dataHi & RDH0R_DATA7_Msk ) >> RDH0R_DATA7_Pos );

    /*-------------------------------------------------
    Let the FIFO know we've read the message so it can
    load in the next one if it exists. Don't leave
    until HW acks the release by clearing the bit.
    -------------------------------------------------*/
    switch( which )
    {
      case Mailbox::RX_MAILBOX_1:
        RFOM0::set( mPeriph, RF0R_RFOM0 );
        while( RFOM0::get( mPeriph ) )
        {
          continue;
        }
        break;

      case Mailbox::RX_MAILBOX_2:
        RFOM1::set( mPeriph, RF1R_RFOM1 );
        while( RFOM1::get( mPeriph ) )
        {
          continue;
        }
        break;

      default:
        // Do nothing. This cannot get hit.
        break;
    }

    /*-------------------------------------------------
    Exit the critical section and return success
    -------------------------------------------------*/
    exitCriticalSection();
    return Chimera::Status::OK;
  }


  /*-------------------------------------------------------------------------------
  Asynchronous Operation
  -------------------------------------------------------------------------------*/
  Chimera::Threading::BinarySemaphore *Driver::getISRSignal( Chimera::CAN::InterruptType signal )
  {
    using namespace Chimera::CAN;
    switch ( signal )
    {
      /*-------------------------------------------------
      Transmit Interrupts
      -------------------------------------------------*/
      // case InterruptType::TX_ISR:
      case InterruptType::TRANSMIT_MAILBOX_EMPTY:
        return &mISREventSignal[ CAN_TX_ISR_SIGNAL_INDEX ];
        break;

      /*-------------------------------------------------
      FIFO Interrupts
      -------------------------------------------------*/
      // case InterruptType::RX_ISR:
      case InterruptType::RECEIVE_FIFO_NEW_MESSAGE:
      case InterruptType::RECEIVE_FIFO_FULL:
      case InterruptType::RECEIVE_FIFO_OVERRUN:
        return &mISREventSignal[ CAN_RX_ISR_SIGNAL_INDEX ];
        break;

      /*-------------------------------------------------
      Status Change
      -------------------------------------------------*/
      // case InterruptType::STS_ISR:
      case InterruptType::SLEEP_EVENT:
      case InterruptType::WAKEUP_EVENT:
        return &mISREventSignal[ CAN_STS_ISR_SIGNAL_INDEX ];
        break;

      /*-------------------------------------------------
      Error Interrupts
      -------------------------------------------------*/
      // case InterruptType::ERR_ISR
      case InterruptType::ERROR_CODE_EVENT:
      case InterruptType::ERROR_BUS_OFF_EVENT:
      case InterruptType::ERROR_PASSIVE_EVENT:
      case InterruptType::ERROR_WARNING_EVENT:
        return &mISREventSignal[ CAN_ERR_ISR_SIGNAL_INDEX ];
        break;

      default:
        return nullptr;
        break;
    };
  }


  const ISREventContext *const Driver::getISRContext( const Chimera::CAN::InterruptType isr )
  {
    using namespace Chimera::CAN;
    switch ( isr )
    {
      /*-------------------------------------------------
      Transmit Interrupts
      -------------------------------------------------*/
      // case InterruptType::TX_ISR:
      case InterruptType::TRANSMIT_MAILBOX_EMPTY:
        return &mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ];
        break;

      /*-------------------------------------------------
      FIFO Interrupts
      -------------------------------------------------*/
      // case InterruptType::RX_ISR:
      case InterruptType::RECEIVE_FIFO_NEW_MESSAGE:
      case InterruptType::RECEIVE_FIFO_FULL:
      case InterruptType::RECEIVE_FIFO_OVERRUN:
        return &mISREventContext[ CAN_RX_ISR_SIGNAL_INDEX ];
        break;

      /*-------------------------------------------------
      Status Change
      -------------------------------------------------*/
      // case InterruptType::STS_ISR:
      case InterruptType::SLEEP_EVENT:
      case InterruptType::WAKEUP_EVENT:
        return &mISREventContext[ CAN_STS_ISR_SIGNAL_INDEX ];
        break;

      /*-------------------------------------------------
      Error Interrupts
      -------------------------------------------------*/
      // case InterruptType::ERR_ISR
      case InterruptType::ERROR_CODE_EVENT:
      case InterruptType::ERROR_BUS_OFF_EVENT:
      case InterruptType::ERROR_PASSIVE_EVENT:
      case InterruptType::ERROR_WARNING_EVENT:
        return &mISREventContext[ CAN_ERR_ISR_SIGNAL_INDEX ];
        break;

      default:
        return nullptr;
        break;
    };
  }


  /*-------------------------------------------------------------------------------
  Protected Functions
  -------------------------------------------------------------------------------*/
  /**
   *  This ISR handles events generated by:
   *    - Transmit mailbox 0/1/2 is empty
   */
  void Driver::CAN1_TX_IRQHandler()
  {
    // Acknowledge the ISR event by clearing RQCP0/1/2 in CAN_TSR

    // give to the tx signal
  }

  /**
   *  This ISR handles events generated by:
   *    - Reception of a new message
   *    - FIFO0 is full
   *    - FIFO0 has overrun
   */
  void Driver::CAN1_FIFO0_IRQHandler()
  {
    // give to the fifo new message signal
  }

  /**
   *  This ISR handles events generated by:
   *    - Reception of a new message
   *    - FIFO1 is full
   *    - FIFO1 has overrun
   */
  void Driver::CAN1_FIFO1_IRQHandler()
  {
    // give to the fifo new message signal
  }

  /**
   *  This ISR handles events generated by:
   *    - Error conditions (multiple kinds)
   *    - Wakeup from SOF monitored on RX pin
   *    - Entry into sleep mode
   */
  void Driver::CAN1_ERR_STS_CHG_IRQHandler()
  {
    // Figure out what fired
    // Give to either status change or error signals
  }


  void Driver::enterCriticalSection()
  {
    for ( auto handlerIdx = 0; handlerIdx < NUM_CAN_IRQ_HANDLERS; handlerIdx++ )
    {
      IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ handlerIdx ] );
    }
  }


  void Driver::exitCriticalSection()
  {
    for ( auto handlerIdx = 0; handlerIdx < NUM_CAN_IRQ_HANDLERS; handlerIdx++ )
    {
      IT::enableIRQ( Resource::IRQSignals[ mResourceIndex ][ handlerIdx ] );
    }
  }

}    // namespace Thor::LLD::CAN

/*-------------------------------------------------------------------------------
Interrupt Vectors
-------------------------------------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
void CAN1_TX_IRQHandler()
{
  using namespace Thor::LLD::CAN;
  s_can_drivers[ CAN1_RESOURCE_INDEX ].CAN1_TX_IRQHandler();
}

void CAN1_FIFO0_IRQHandler()
{
  using namespace Thor::LLD::CAN;
  s_can_drivers[ CAN1_RESOURCE_INDEX ].CAN1_FIFO0_IRQHandler();
}

void CAN1_FIFO1_IRQHandler()
{
  using namespace Thor::LLD::CAN;
  s_can_drivers[ CAN1_RESOURCE_INDEX ].CAN1_FIFO1_IRQHandler();
}

void CAN1_ERR_STS_CHG_IRQHandler()
{
  using namespace Thor::LLD::CAN;
  s_can_drivers[ CAN1_RESOURCE_INDEX ].CAN1_ERR_STS_CHG_IRQHandler();
}
#endif

#endif /* TARGET_STM32L4 & THOR_LLD_CAN */
