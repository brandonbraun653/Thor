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
#include <Thor/lld/stm32l4x/can/hw_can_prv_driver.hpp>
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
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), mResourceIndex( std::numeric_limits<size_t>::max() )
  {
  }


  Driver::~Driver()
  {
    /*-------------------------------------------------
    Disable peripheral level ISR signals
    -------------------------------------------------*/
    for ( auto signalIdx = 0; signalIdx < static_cast<size_t>( Chimera::CAN::InterruptType::NUM_OPTIONS ); signalIdx++ )
    {
      disableISRSignal( static_cast<Chimera::CAN::InterruptType>( signalIdx ) );
    }

    /*-------------------------------------------------
    Reduce power consumption
    -------------------------------------------------*/
    disableClock();
  }

  /*-------------------------------------------------------------------------------
  Configuration
  -------------------------------------------------------------------------------*/
  void Driver::attach( RegisterMap *const peripheral )
  {
    using namespace Chimera::CAN;

    /*------------------------------------------------
    Get peripheral descriptor settings
    ------------------------------------------------*/
    mPeriph        = peripheral;
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );

    /*------------------------------------------------
    System level ISR configuration
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

    /*-------------------------------------------------
    Disable peripheral level ISR signals
    -------------------------------------------------*/
    for ( auto signalIdx = 0; signalIdx < static_cast<size_t>( InterruptType::NUM_OPTIONS ); signalIdx++ )
    {
      disableISRSignal( static_cast<InterruptType>( signalIdx ) );
    }
  }


  void Driver::enableClock()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_CAN, mResourceIndex );
  }


  void Driver::disableClock()
  {
    auto rcc = Thor::LLD::RCC::getPeripheralClock();
    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_CAN, mResourceIndex );
  }


  Chimera::Status_t Driver::configure( const Chimera::CAN::DriverConfig &cfg )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Initialize the GPIO drivers
    -------------------------------------------------*/
    auto txPin = Chimera::GPIO::getDriver( cfg.TXInit.port, cfg.TXInit.pin );
    auto rxPin = Chimera::GPIO::getDriver( cfg.RXInit.port, cfg.RXInit.pin );

    if ( !txPin || !rxPin )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    auto pinConfigError = Chimera::Status::OK;
    pinConfigError |= txPin->init( cfg.TXInit );
    pinConfigError |= rxPin->init( cfg.RXInit );

    if ( pinConfigError != Chimera::Status::OK )
    {
      return Chimera::Status::FAILED_INIT;
    }

    /*-------------------------------------------------
    Reset the driver registers to default values then
    configure hardware for initialization.
    -------------------------------------------------*/
    prv_reset( mPeriph );
    prv_enter_initialization_mode( mPeriph );

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
    Set up the resync jump width
    -------------------------------------------------*/
    Reg32_t shiftedJumpWidth = ( cfg.HWInit.resyncJumpWidth & BTR_SJW_Wid ) << BTR_SJW_Pos;
    SJW::set( mPeriph, shiftedJumpWidth );

    /*-------------------------------------------------
    Request to leave init mode
    -------------------------------------------------*/
    prv_enter_normal_mode( mPeriph );
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::applyFilters( MessageFilter *const filterList, const size_t filterSize )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( !filterList || !filterSize || ( filterSize > NUM_CAN_MAX_FILTERS ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Sort the filters by how much space they consume.
    This allows for an easier placement algorithm.
    -------------------------------------------------*/
    uint8_t sortedFilterIdx[ NUM_CAN_MAX_FILTERS ];
    CLEAR_ARRAY( sortedFilterIdx );

    if ( !sortFiltersBySize( filterList, filterSize, sortedFilterIdx ) )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------
    Configuration requires being in initialization mode
    and the FINIT bit set in the CAN_FMR register.
    -------------------------------------------------*/
    prv_enter_initialization_mode( mPeriph );
    FINIT::set( mPeriph, FMR_FINIT );

    /*-------------------------------------------------
    Reset the entire filter configuration
    -------------------------------------------------*/
    mPeriph->FM1R  = 0;
    mPeriph->FS1R  = 0;
    mPeriph->FFA1R = 0;
    mPeriph->FA1R  = 0;

    for ( uint8_t bankIdx = 0; bankIdx < NUM_CAN_FILTER_BANKS; bankIdx++ )
    {
      // Use magic values to help figure out in-use filters
      mPeriph->sFilterBank[ bankIdx ].FR1 = FLTR_RST_1;
      mPeriph->sFilterBank[ bankIdx ].FR2 = FLTR_RST_2;
    }

    /*-------------------------------------------------
    Attempt to place each filter in the filter banks
    -------------------------------------------------*/
    bool bankConfigured       = false;      // Tracks if the current bank has been configured already
    size_t bankIdx            = 0;          // Which hardware filter bank currently being processed
    size_t userIdx            = 0;          // Which user filter currently being processed
    size_t fltrIdx            = 0;          // Generic index tracking progress of filter processing
    size_t numValidFilters    = 0;          // Number of valid filter configurations attempted to be placed
    size_t numPlacedFilters   = 0;          // Number of filters that were successfully placed
    uint8_t fifo0FMI_current  = 0;          // Current filter match index for FIFO0
    uint8_t fifo1FMI_current  = 0;          // Current filter match index for FIFO1
    uint8_t fifo0FMI_toAssign = 0;          // Remaining match indices to assign on the current filter bank
    uint8_t fifo1FMI_toAssign = 0;          // Remaining match indices to assign on the current filter bank
    MessageFilter *filter     = nullptr;    // Pointer to the current user filter
    volatile FilterReg *bank  = nullptr;    // Current bank being processed


    do
    {
      /*-------------------------------------------------
      Grab the highest priority filter to place next
      -------------------------------------------------*/
      userIdx = sortedFilterIdx[ fltrIdx ];
      filter  = &filterList[ userIdx ];
      bank    = &mPeriph->sFilterBank[ bankIdx ];

      /*-------------------------------------------------
      Is the filter even marked as valid? No point trying
      to place it unless it is.
      -------------------------------------------------*/
      if( !filter->valid )
      {
        fltrIdx++;
        continue;
      }

      /*-------------------------------------------------
      Is the filter the right type to be placed in this
      filter bank? Can't mix filter modes willy nilly. A
      new mode requires a new filter bank.
      -------------------------------------------------*/
      if( bankConfigured && ( filter->mode != prv_get_filter_bank_mode( mPeriph, bankIdx ) ) )
      {
        /*-------------------------------------------------
        FMI is linear, so if the bank is partially assigned
        then the remaining unassigned filters are skipped.
        The skipping needs to be tracked.
        -------------------------------------------------*/
        if( prv_get_filter_bank_fifo( mPeriph, bankIdx ) == Mailbox::RX_MAILBOX_1 )
        {
          fifo0FMI_current += fifo0FMI_toAssign;
          fifo0FMI_toAssign = 0;
        }
        else
        {
          fifo1FMI_current += fifo1FMI_toAssign;
          fifo1FMI_toAssign = 0;
        }

        /*-------------------------------------------------
        Reset the trackers for the next bank
        -------------------------------------------------*/
        bankIdx++;
        bankConfigured = false;
        continue;
      }
      else
      {
        /*-------------------------------------------------
        New bank! The next filter is guaranteed to fit
        because it determines the bank properties.

        Reset the FMI counters based on the new mode.
        -------------------------------------------------*/
        uint8_t *fmi = &fifo0FMI_toAssign;
        if( filter->fifoBank == Mailbox::RX_MAILBOX_2 )
        {
          fmi = &fifo1FMI_toAssign;
        }

        switch( filter->mode)
        {
          case FilterMode::MODE_16BIT_LIST:
            *fmi = 4;
            break;

          case FilterMode::MODE_16BIT_MASK:
          case FilterMode::MODE_32BIT_LIST:
            *fmi = 2;
            break;

          case FilterMode::MODE_32BIT_MASK:
            *fmi = 1;
            break;

          default:
            return Chimera::Status::FAIL;
            break;
        };
      }

      /*-------------------------------------------------
      Alright, so the filter is the correct type, but is
      there room in the bank to hold it? If not, move on
      to the next bank.
      -------------------------------------------------*/
      auto nextSlot = FilterSlot::UNKNOWN;

      if ( prv_does_filter_fit( filter, bank, nextSlot ) )
      {
        /*-------------------------------------------------
        Phew, all the checks passed. Now to actually start
        assigning the filter into hardware.
        -------------------------------------------------*/
        numValidFilters++;
        prv_assign_filter( filter, bank, nextSlot );
      }
      else
      {
        /*-------------------------------------------------
        Move on to the next bank. This one is out of room.
        -------------------------------------------------*/
        bankIdx++;
        bankConfigured = false;
        continue;
      }

      /*-------------------------------------------------
      Configure Mask/List Mode & Scale
      -------------------------------------------------*/
      switch ( filter->mode )
      {
        case Chimera::CAN::FilterMode::MODE_16BIT_LIST:
          mPeriph->FM1R |= ( 1u << bankIdx );     // Mask
          mPeriph->FS1R &= ~( 1u << bankIdx );    // Scale
          break;

        case Chimera::CAN::FilterMode::MODE_16BIT_MASK:
          mPeriph->FM1R &= ~( 1u << bankIdx );    // Mask
          mPeriph->FS1R &= ~( 1u << bankIdx );    // Scale
          break;

        case Chimera::CAN::FilterMode::MODE_32BIT_LIST:
          mPeriph->FM1R |= ( 1u << bankIdx );     // Mask
          mPeriph->FS1R |= ( 1u << bankIdx );     // Scale
          break;

        case Chimera::CAN::FilterMode::MODE_32BIT_MASK:
          mPeriph->FM1R &= ~( 1u << bankIdx );    // Mask
          mPeriph->FS1R |= ( 1u << bankIdx );     // Scale
          break;

        default:
          return Chimera::Status::NOT_INITIALIZED;
          break;
      };

      /*-------------------------------------------------
      Configure FIFO Assignment
      -------------------------------------------------*/
      switch ( filter->fifoBank )
      {
        case Mailbox::RX_MAILBOX_1:
          mPeriph->FFA1R &= ~( 1u << bankIdx );
          filter->assignedFMI = fifo0FMI_current;
          fifo0FMI_current++;
          fifo0FMI_toAssign--;
          break;

        case Mailbox::RX_MAILBOX_2:
          mPeriph->FFA1R |= ( 1u << bankIdx );
          filter->assignedFMI = fifo1FMI_current;
          fifo1FMI_current++;
          fifo1FMI_toAssign--;
          break;

        default:
          return Chimera::Status::NOT_INITIALIZED;
          break;
      };

      /*-------------------------------------------------
      Configure Activation State
      -------------------------------------------------*/
      if ( filter->active )
      {
        mPeriph->FA1R |= ( 1u << bankIdx );
      }
      else
      {
        mPeriph->FA1R &= ~( 1u << bankIdx );
      }

      /*-------------------------------------------------
      Update tracking data
      -------------------------------------------------*/
      fltrIdx++;              // Just completed parsing one of the user's filters
      numPlacedFilters++;     // Just completed parsing a valid filter
      bankConfigured = true;  // The current filter bank has been configured at least once now

    } while ( ( bankIdx < NUM_CAN_FILTER_BANKS ) && ( fltrIdx < filterSize ) );

    /*-------------------------------------------------
    Leave filter initialization mode
    -------------------------------------------------*/
    FINIT::clear( mPeriph, FMR_FINIT );
    prv_enter_normal_mode( mPeriph );

    /*-------------------------------------------------
    Were all the filters placed?
    -------------------------------------------------*/
    if ( numPlacedFilters == numValidFilters )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FULL;
    }
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

    return Chimera::Status::OK;
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
        TMEIE::clear( mPeriph, IER_TMEIE );
        break;

      /*-------------------------------------------------
      FIFO Interrupts
      -------------------------------------------------*/
      case InterruptType::RECEIVE_FIFO_NEW_MESSAGE:
        FMPIE0::clear( mPeriph, IER_FMPIE0 );
        FMPIE1::clear( mPeriph, IER_FMPIE1 );
        break;

      case InterruptType::RECEIVE_FIFO_FULL:
        FFIE0::clear( mPeriph, IER_FFIE0 );
        FFIE1::clear( mPeriph, IER_FFIE1 );
        break;

      case InterruptType::RECEIVE_FIFO_OVERRUN:
        FOVIE0::clear( mPeriph, IER_FOVIE0 );
        FOVIE1::clear( mPeriph, IER_FOVIE1 );
        break;

      /*-------------------------------------------------
      Status Change Interrupts
      -------------------------------------------------*/
      case InterruptType::SLEEP_EVENT:
        SLKIE::clear( mPeriph, IER_SLKIE );
        break;

      case InterruptType::WAKEUP_EVENT:
        WKUIE::clear( mPeriph, IER_WKUIE );
        break;

      /*-------------------------------------------------
      Error Interrupts
      -------------------------------------------------*/
      case InterruptType::ERROR_CODE_EVENT:
        LECIE::clear( mPeriph, IER_LECIE );
        break;

      case InterruptType::ERROR_BUS_OFF_EVENT:
        BOFIE::clear( mPeriph, IER_BOFIE );
        break;

      case InterruptType::ERROR_PASSIVE_EVENT:
        EPVIE::clear( mPeriph, IER_EPVIE );
        break;

      case InterruptType::ERROR_WARNING_EVENT:
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
    if ( !prv_validate_frame( frame ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Make sure we are in normal mode
    -------------------------------------------------*/
    if ( !prv_in_normal_mode( mPeriph ) )
    {
      prv_enter_normal_mode( mPeriph );
    }

    /*-------------------------------------------------
    Enter a critical section to guarantee the mailbox
    will only be modified by this function.
    -------------------------------------------------*/
    enterCriticalSection();

    /*-------------------------------------------------
    Validate that the mailbox is actually free
    -------------------------------------------------*/
    volatile TxMailbox *box = nullptr;

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
    if ( frame.idMode == IdentifierMode::STANDARD )
    {
      box->TIR |= ( frame.id & ID_MASK_11_BIT ) << TIxR_STID_Pos;
      box->TIR &= ~TIxR_IDE;
    }
    else    // Extended
    {
      box->TIR |= ( ( frame.id & ID_MASK_29_BIT ) << TIxR_EXID_Pos );
      box->TIR |= TIxR_IDE;
    }

    /*-------------------------------------------------
    Assign the remote transmition type
    -------------------------------------------------*/
    if ( frame.frameType == FrameType::DATA )
    {
      box->TIR &= ~TIxR_RTR;
    }
    else    // Remote frame
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
    volatile RxMailbox *box = nullptr;

    switch ( which )
    {
      case Mailbox::RX_MAILBOX_1:
        if ( !FMP0::get( mPeriph ) )
        {
          exitCriticalSection();
          return Chimera::Status::NOT_READY;
        }

        box = &mPeriph->sFIFOMailBox[ RIDX_RX_MAILBOX_1 ];
        break;

      case Mailbox::RX_MAILBOX_2:
        if ( !FMP1::get( mPeriph ) )
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
    if ( !( box->RIR & RI0R_IDE ) )
    {
      frame.id = ( ( box->RIR & RI0R_STID_Msk ) >> RI0R_STID_Pos );
    }
    else    // Extended
    {
      frame.id = ( ( box->RIR & RI0R_EXID_Msk ) >> RI0R_EXID_Pos );
    }

    /*-------------------------------------------------
    Read out the frame type
    -------------------------------------------------*/
    if ( !( box->RIR & RI0R_RTR ) )
    {
      frame.frameType = FrameType::DATA;
    }
    else    // Remote
    {
      frame.frameType = FrameType::REMOTE;
    }

    /*-------------------------------------------------
    Read out the filter match idx and payload length
    -------------------------------------------------*/
    frame.filterIndex = ( ( box->RDTR & RDT0R_FMI_Msk ) >> RDT0R_FMI_Pos );
    frame.dataLength  = ( ( box->RDTR & RDT0R_DLC_Msk ) >> RDT0R_DLC_Pos );

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
    switch ( which )
    {
      case Mailbox::RX_MAILBOX_1:
        RFOM0::set( mPeriph, RF0R_RFOM0 );
        while ( RFOM0::get( mPeriph ) )
        {
          continue;
        }
        break;

      case Mailbox::RX_MAILBOX_2:
        RFOM1::set( mPeriph, RF1R_RFOM1 );
        while ( RFOM1::get( mPeriph ) )
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
        Chimera::insert_debug_breakpoint();
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


  void Driver::setISRHandled( const Chimera::CAN::InterruptType isr )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    For now, all this means is that it's ok to reenable
    the given ISR signal. The event was handled ok.
    -------------------------------------------------*/
    enterCriticalSection();
    enableISRSignal( isr );

    auto isrBit = ( 1u << static_cast<size_t>( isr ) );

    switch ( isr )
    {
      /*-------------------------------------------------
      Transmit Interrupts
      -------------------------------------------------*/
      case InterruptType::TRANSMIT_MAILBOX_EMPTY:
        mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].isrPending &= ~isrBit;
        break;

      /*-------------------------------------------------
      FIFO Interrupts
      -------------------------------------------------*/
      case InterruptType::RECEIVE_FIFO_NEW_MESSAGE:
      case InterruptType::RECEIVE_FIFO_FULL:
      case InterruptType::RECEIVE_FIFO_OVERRUN:
        mISREventContext[ CAN_RX_ISR_SIGNAL_INDEX ].isrPending &= ~isrBit;
        break;

      /*-------------------------------------------------
      Status Change Interrupts
      -------------------------------------------------*/
      case InterruptType::SLEEP_EVENT:
      case InterruptType::WAKEUP_EVENT:
        mISREventContext[ CAN_STS_ISR_SIGNAL_INDEX ].isrPending &= ~isrBit;
        break;

      /*-------------------------------------------------
      Error Interrupts
      -------------------------------------------------*/
      case InterruptType::ERROR_CODE_EVENT:
      case InterruptType::ERROR_BUS_OFF_EVENT:
      case InterruptType::ERROR_PASSIVE_EVENT:
      case InterruptType::ERROR_WARNING_EVENT:
        mISREventContext[ CAN_ERR_ISR_SIGNAL_INDEX ].isrPending &= ~isrBit;
        break;

      default:
        // Do nothing, it's not supported.
        break;
    };

    exitCriticalSection();
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
    /*-------------------------------------------------
    Ensure the ISR type is set correctly
    -------------------------------------------------*/
    mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].isrPending |=
        ( 1u << static_cast<size_t>( Chimera::CAN::InterruptType::TRANSMIT_MAILBOX_EMPTY ) );

    /*-------------------------------------------------
    Read the latest value of the transmit status register
    -------------------------------------------------*/
    const Reg32_t tsr = mPeriph->TSR;

    /*-------------------------------------------------
    Mailbox 0 Transmit Complete
    -------------------------------------------------*/
    if ( tsr & TSR_RQCP0 )
    {
      // Copy out the results of the last event
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 0 ].txError = static_cast<bool>( TERR0::get( mPeriph ) );
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 0 ].arbLost = static_cast<bool>( ALST0::get( mPeriph ) );
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 0 ].txOk    = static_cast<bool>( TXOK0::get( mPeriph ) );

      // Acknowledge event. Setting this bit clears the above registers.
      RQCP0::set( mPeriph, TSR_RQCP0 );
    }

    /*-------------------------------------------------
    Mailbox 1 Transmit Complete
    -------------------------------------------------*/
    if ( tsr & TSR_RQCP1 )
    {
      // Copy out the results of the last event
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 1 ].txError = static_cast<bool>( TERR1::get( mPeriph ) );
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 1 ].arbLost = static_cast<bool>( ALST1::get( mPeriph ) );
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 1 ].txOk    = static_cast<bool>( TXOK1::get( mPeriph ) );

      // Acknowledge event. Setting this bit clears the above registers.
      RQCP1::set( mPeriph, TSR_RQCP1 );
    }

    /*-------------------------------------------------
    Mailbox 2 Transmit Complete
    -------------------------------------------------*/
    if ( tsr & TSR_RQCP2 )
    {
      // Copy out the results of the last event
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 2 ].txError = static_cast<bool>( TERR2::get( mPeriph ) );
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 2 ].arbLost = static_cast<bool>( ALST2::get( mPeriph ) );
      mISREventContext[ CAN_TX_ISR_SIGNAL_INDEX ].event.tx[ 2 ].txOk    = static_cast<bool>( TXOK2::get( mPeriph ) );

      // Acknowledge event. Setting this bit clears the above registers.
      RQCP2::set( mPeriph, TSR_RQCP2 );
    }

    /*-------------------------------------------------
    Awaken high priority thread for processing this ISR
    -------------------------------------------------*/
    mISREventSignal[ CAN_TX_ISR_SIGNAL_INDEX ].releaseFromISR();
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
    /*-------------------------------------------------
    Define flags for detecting event categories
    -------------------------------------------------*/
    constexpr Reg32_t IER_EN_STS     = ( IER_WKUIE | IER_SLKIE );
    constexpr Reg32_t IER_EN_ERR_ALL = ( IER_ERRIE );
    constexpr Reg32_t IER_EN_ERR     = ( IER_EWGIE | IER_EPVIE | IER_BOFIE | IER_LECIE );
    constexpr Reg32_t EVENT_STS      = ( MSR_WKUI | MSR_SLAKI );
    constexpr Reg32_t EVENT_ERR      = ( ESR_EWGF | ESR_EPVF | ESR_BOFF | ESR_LEC );

    /*-------------------------------------------------
    Read the relevant status registers
    -------------------------------------------------*/
    const Reg32_t IER = IER_ALL::get( mPeriph );    // Interrupt enable register
    const Reg32_t ESR = ESR_ALL::get( mPeriph );    // Error status register
    const Reg32_t MSR = MSR_ALL::get( mPeriph );    // Master status register

    /*-------------------------------------------------
    Handle Status Change Events
    -------------------------------------------------*/
    if ( ( IER & IER_EN_STS ) && ( MSR & EVENT_STS ) )
    {
      constexpr size_t sigIdx = CAN_STS_ISR_SIGNAL_INDEX;
      bool hpThreadShouldWake = false;

      /*-------------------------------------------------
      Parse Sleep Event
      -------------------------------------------------*/
      constexpr uint16_t sleepBit = ( 1u << static_cast<size_t>( Chimera::CAN::InterruptType::SLEEP_EVENT ) );

      if ( ( IER & IER_SLKIE ) && ( MSR & MSR_SLAKI ) )
      {
        disableISRSignal( Chimera::CAN::InterruptType::SLEEP_EVENT );
        mISREventContext[ sigIdx ].event.sts.sleepAck = true;
        mISREventContext[ sigIdx ].isrPending |= sleepBit;
        hpThreadShouldWake = true;
      }
      else if ( !( mISREventContext[ sigIdx ].isrPending & sleepBit ) )
      {
        mISREventContext[ sigIdx ].event.sts.sleepAck = false;
      }

      /*-------------------------------------------------
      Parse Wakeup Event
      -------------------------------------------------*/
      constexpr uint16_t wakeupBit = ( 1u << static_cast<size_t>( Chimera::CAN::InterruptType::WAKEUP_EVENT ) );

      if ( ( IER & IER_WKUIE ) && ( MSR & MSR_WKUI ) )
      {
        disableISRSignal( Chimera::CAN::InterruptType::WAKEUP_EVENT );
        mISREventContext[ sigIdx ].event.sts.wakeup = true;
        mISREventContext[ sigIdx ].isrPending |= wakeupBit;
        hpThreadShouldWake = true;
      }
      else if ( !( mISREventContext[ sigIdx ].isrPending & wakeupBit ) )
      {
        mISREventContext[ sigIdx ].event.sts.wakeup = false;
      }

      /*-------------------------------------------------
      Awaken high priority thread for processing this ISR
      -------------------------------------------------*/
      if ( hpThreadShouldWake )
      {
        mISREventSignal[ CAN_STS_ISR_SIGNAL_INDEX ].releaseFromISR();
      }
    }

    /*-------------------------------------------------
    Handle Error Events
    -------------------------------------------------*/
    if ( ( IER & IER_EN_ERR_ALL ) && ( IER & IER_EN_ERR ) && ( ESR & EVENT_ERR ) )
    {
      constexpr size_t sigIdx = CAN_ERR_ISR_SIGNAL_INDEX;
      bool hpThreadShouldWake = false;

      /*-------------------------------------------------
      Acknowledge that we hit the error ISR. This is done
      by setting the bit as opposed to clearing.
      -------------------------------------------------*/
      ERRI::set( mPeriph, MSR_ERRI );

      /*-------------------------------------------------
      Update the error counters
      -------------------------------------------------*/
      mISREventContext[ sigIdx ].event.err.txErrorCount = static_cast<uint8_t>( ( ( ESR & ESR_TEC ) >> ESR_TEC_Pos ) );
      mISREventContext[ sigIdx ].event.err.rxErrorCount = static_cast<uint8_t>( ( ( ESR & ESR_REC ) >> ESR_REC_Pos ) );

      /*-------------------------------------------------
      Parse Warning Errors
      -------------------------------------------------*/
      constexpr uint16_t warningBit = ( 1u << static_cast<size_t>( Chimera::CAN::InterruptType::ERROR_WARNING_EVENT ) );

      if ( ( IER & IER_EWGIE ) && ( ESR & ESR_EWGF ) )
      {
        disableISRSignal( Chimera::CAN::InterruptType::ERROR_WARNING_EVENT );
        mISREventContext[ sigIdx ].event.err.warning = true;
        mISREventContext[ sigIdx ].isrPending |= warningBit;
        hpThreadShouldWake = true;
      }
      else if ( !( mISREventContext[ sigIdx ].isrPending & warningBit ) )
      {
        mISREventContext[ sigIdx ].event.err.warning = false;
      }
      // else higher priority thread hasn't handled the event yet

      /*-------------------------------------------------
      Parse Passive Errors
      -------------------------------------------------*/
      constexpr uint16_t passiveBit = ( 1u << static_cast<size_t>( Chimera::CAN::InterruptType::ERROR_PASSIVE_EVENT ) );

      if ( ( IER & IER_EPVIE ) && ( ESR & ESR_EPVF ) )
      {
        disableISRSignal( Chimera::CAN::InterruptType::ERROR_PASSIVE_EVENT );
        mISREventContext[ sigIdx ].event.err.passive = true;
        mISREventContext[ sigIdx ].isrPending |= passiveBit;
        hpThreadShouldWake = true;
      }
      else if ( !( mISREventContext[ sigIdx ].isrPending & passiveBit ) )
      {
        mISREventContext[ sigIdx ].event.err.passive = false;
      }
      // else higher priority thread hasn't handled the event yet

      /*-------------------------------------------------
      Parse Bus Off Errors
      -------------------------------------------------*/
      constexpr uint16_t busOffBit = ( 1u << static_cast<size_t>( Chimera::CAN::InterruptType::ERROR_BUS_OFF_EVENT ) );

      if ( ( IER & IER_BOFIE ) && ( ESR & ESR_BOFF ) )
      {
        disableISRSignal( Chimera::CAN::InterruptType::ERROR_BUS_OFF_EVENT );
        mISREventContext[ sigIdx ].event.err.busOff = true;
        mISREventContext[ sigIdx ].isrPending |= busOffBit;
        hpThreadShouldWake = true;
      }
      else if ( !( mISREventContext[ sigIdx ].isrPending & busOffBit ) )
      {
        mISREventContext[ sigIdx ].event.err.busOff = false;
      }

      /*-------------------------------------------------
      Parse Last Error Code Errors
      -------------------------------------------------*/
      constexpr uint16_t errorCodeBit = ( 1u << static_cast<size_t>( Chimera::CAN::InterruptType::ERROR_CODE_EVENT ) );

      if ( ( IER & IER_LECIE ) && ( ESR & ESR_LEC ) )
      {
        disableISRSignal( Chimera::CAN::InterruptType::ERROR_CODE_EVENT );
        mISREventContext[ sigIdx ].event.err.lastErrorCode = static_cast<ErrorCode>( ( ESR & ESR_LEC ) >> ESR_LEC_Pos );
        mISREventContext[ sigIdx ].isrPending |= errorCodeBit;
        hpThreadShouldWake = true;
      }
      else if ( !( mISREventContext[ sigIdx ].isrPending & errorCodeBit ) )
      {
        mISREventContext[ sigIdx ].event.err.lastErrorCode = ErrorCode::NO_ERROR;
      }
      // else higher priority thread hasn't handled the event yet

      /*-------------------------------------------------
      Awaken high priority thread for processing this ISR
      -------------------------------------------------*/
      if ( hpThreadShouldWake )
      {
        mISREventSignal[ sigIdx ].releaseFromISR();
      }
    }
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

void CAN1_RX0_IRQHandler()
{
  using namespace Thor::LLD::CAN;
  s_can_drivers[ CAN1_RESOURCE_INDEX ].CAN1_FIFO0_IRQHandler();
}

void CAN1_RX1_IRQHandler()
{
  using namespace Thor::LLD::CAN;
  s_can_drivers[ CAN1_RESOURCE_INDEX ].CAN1_FIFO1_IRQHandler();
}

void CAN1_SCE_IRQHandler()
{
  using namespace Thor::LLD::CAN;
  s_can_drivers[ CAN1_RESOURCE_INDEX ].CAN1_ERR_STS_CHG_IRQHandler();
}
#endif

#endif /* TARGET_STM32L4 & THOR_LLD_CAN */
