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


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() :
      mPeriph( nullptr ), mResourceIndex( std::numeric_limits<size_t>::max() ), mISRWakeup_external( nullptr ),
      mTXPin( nullptr ), mRXPin( nullptr )
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
      Thor::LLD::IT::disableIRQ( Resource::IRQSignals[ mResourceIndex ][ handlerIdx ] );
      Thor::LLD::IT::clearPendingIRQ( Resource::IRQSignals[ mResourceIndex ][ handlerIdx ] );
      Thor::LLD::IT::setPriority( Resource::IRQSignals[ mResourceIndex ][ handlerIdx ],
                                  Thor::Interrupt::CAN_IT_PREEMPT_PRIORITY, 0u );
    }

    return Chimera::Status::OK;
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
    switch( signal )
    {
      /*-------------------------------------------------
      Transmit Interrupts
      -------------------------------------------------*/
      case InterruptType::TRANSMIT_MAILBOX_EMPTY:

        break;

      /*-------------------------------------------------
      FIFO Interrupts
      -------------------------------------------------*/
      case InterruptType::RECEIVE_FIFO_NEW_MESSAGE:

        break;

      case InterruptType::RECEIVE_FIFO_FULL:

        break;

      case InterruptType::RECEIVE_FIFO_OVERRUN:

        break;

      /*-------------------------------------------------
      Status Change Interrupts
      -------------------------------------------------*/
      case InterruptType::SLEEP_EVENT:

        break;

      case InterruptType::WAKEUP_EVENT:

        break;

      /*-------------------------------------------------
      Error Interrupts
      -------------------------------------------------*/
      case InterruptType::ERROR_PENDING:

        break;

      case InterruptType::ERROR_CODE_EVENT:

        break;

      case InterruptType::ERROR_BUS_OFF_EVENT:

        break;

      case InterruptType::ERROR_PASSIVE_EVENT:

        break;

      case InterruptType::ERROR_WARNING_EVENT:

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
    switch( signal )
    {
      /*-------------------------------------------------
      Transmit Interrupts
      -------------------------------------------------*/
      case InterruptType::TRANSMIT_MAILBOX_EMPTY:

        break;

      /*-------------------------------------------------
      FIFO Interrupts
      -------------------------------------------------*/
      case InterruptType::RECEIVE_FIFO_NEW_MESSAGE:

        break;

      case InterruptType::RECEIVE_FIFO_FULL:

        break;

      case InterruptType::RECEIVE_FIFO_OVERRUN:

        break;

      /*-------------------------------------------------
      Status Change Interrupts
      -------------------------------------------------*/
      case InterruptType::SLEEP_EVENT:

        break;

      case InterruptType::WAKEUP_EVENT:

        break;

      /*-------------------------------------------------
      Error Interrupts
      -------------------------------------------------*/
      case InterruptType::ERROR_PENDING:

        break;

      case InterruptType::ERROR_CODE_EVENT:

        break;

      case InterruptType::ERROR_BUS_OFF_EVENT:

        break;

      case InterruptType::ERROR_PASSIVE_EVENT:

        break;

      case InterruptType::ERROR_WARNING_EVENT:

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
    switch( mode )
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

    if( fifo0PendingMessages )
    {
      which = Mailbox::RX_MAILBOX_1;
      return true;
    }
    else if( fifo1PendingMessages )
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
  }


  Chimera::Status_t Driver::receive( const Mailbox which, Chimera::CAN::BasicFrame &frame )
  {
  }


  /*-------------------------------------------------------------------------------
  Asynchronous Operation
  -------------------------------------------------------------------------------*/
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
