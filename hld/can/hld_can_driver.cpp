/********************************************************************************
 *  File Name:
 *    hld_can_driver.cpp
 *
 *  Description:
 *    CAN driver for Thor
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>
#include <limits>

/* Aurora Includes */
#include <Aurora/constants>

/* Chimera Includes */
#include <Chimera/can>
#include <Chimera/thread>
#include <Chimera/event>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/can>
#include <Thor/lld/interface/can/can_detail.hpp>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_types.hpp>

#if defined( THOR_HLD_CAN )

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::CAN;
namespace LLD = ::Thor::LLD::CAN;

using ThreadHandle = Chimera::Thread::detail::native_thread_handle_type;
using BinarySemphr = Chimera::Thread::BinarySemaphore;
using ThreadFunctn = Chimera::Function::void_func_void_ptr;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = LLD::NUM_CAN_PERIPHS;
static constexpr size_t NUM_ISR_SIG = LLD::NUM_CAN_IRQ_HANDLERS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static size_t s_driver_initialized;

/*-------------------------------------------------
Instances of the CAN driver in object and ptr form
-------------------------------------------------*/
static HLD::Driver hld_driver[ NUM_DRIVERS ];

/*-------------------------------------------------
High priority threads & handles that process more
complex ISR functionality.
-------------------------------------------------*/
static ThreadHandle s_user_isr_handle[ NUM_DRIVERS ][ NUM_ISR_SIG ];
static ThreadFunctn s_user_isr_thread_func[ NUM_DRIVERS ][ NUM_ISR_SIG ];

/*-------------------------------------------------
Wakeup signals used to allow external threads to
pend on interrupt processing events.
-------------------------------------------------*/
static BinarySemphr s_user_isr_signal[ NUM_DRIVERS ][ NUM_ISR_SIG ];

/*-------------------------------------------------------------------------------
Private Function Declarations
-------------------------------------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
static void CAN1ISR_TXHandler( void *argument );
static void CAN1ISR_RXHandler( void *argument );
static void CAN1ISR_StatusChangeHandler( void *argument );
static void CAN1ISR_ErrHandler( void *argument );
#endif

namespace Thor::CAN
{
  using namespace Chimera::Thread;

  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
  static bool validateCfg( const Chimera::CAN::DriverConfig &cfg )
  {
    /*-------------------------------------------------
    At a bare minimum, if the user didn't set the actual
    validity flag, it's going to mark all these checks
    as invalid.
    -------------------------------------------------*/
    bool result = cfg.validity;

    /*-------------------------------------------------
    User needs to validate the pin configs too
    -------------------------------------------------*/
    result = result && cfg.RXInit.validity;
    result = result && cfg.TXInit.validity;

    /*-------------------------------------------------
    The buffers must be pre-allocated as these drivers
    do not use dynamic memory. If dynamic memory is
    desired, it must be allocated externally.
    -------------------------------------------------*/
    result = result && ( cfg.HWInit.rxBuffer != nullptr );
    result = result && ( cfg.HWInit.txBuffer != nullptr );
    result = result && ( cfg.HWInit.rxElements != 0 );
    result = result && ( cfg.HWInit.txElements != 0 );

    /*-------------------------------------------------
    Last but not least, does the channel even exist?
    -------------------------------------------------*/
    result = result && ( cfg.HWInit.channel < Chimera::CAN::Channel::NUM_OPTIONS );

    return result;
  }

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*------------------------------------------------
    Prevent multiple initializations (need reset first)
    ------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    auto result = ::LLD::initialize();

    /*------------------------------------------------
    Initialize ISR post-processing routines
    ------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::CAN1_RESOURCE_INDEX ][::LLD::CAN_TX_ISR_SIGNAL_INDEX ]  = CAN1ISR_TXHandler;
    s_user_isr_thread_func[::LLD::CAN1_RESOURCE_INDEX ][::LLD::CAN_RX_ISR_SIGNAL_INDEX ]  = CAN1ISR_RXHandler;
    s_user_isr_thread_func[::LLD::CAN1_RESOURCE_INDEX ][::LLD::CAN_STS_ISR_SIGNAL_INDEX ] = CAN1ISR_StatusChangeHandler;
    s_user_isr_thread_func[::LLD::CAN1_RESOURCE_INDEX ][::LLD::CAN_ERR_ISR_SIGNAL_INDEX ] = CAN1ISR_ErrHandler;
#endif

    /*-------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }


  Chimera::Status_t reset()
  {
    /*------------------------------------------------
    Only allow clearing of local data during testing
    ------------------------------------------------*/
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_CAN )
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    for ( auto x = 0; x < NUM_DRIVERS; x++ )
    {
      hld_shared[ x ].reset();
    }
#endif

    return Chimera::Status::OK;
  }


  Driver_rPtr getDriver( const Chimera::CAN::Channel channel )
  {
    auto idx = ::LLD::getResourceIndex( channel );
    if ( ( idx != ::Thor::LLD::INVALID_RESOURCE_INDEX ) && ( idx < NUM_DRIVERS ) )
    {
      return &hld_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver()
  {
    mConfig.clear();
  }


  Driver::~Driver()
  {
  }


  /*------------------------------------------------
  HW Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::open( const Chimera::CAN::DriverConfig &cfg )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( !validateCfg( cfg ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    // Only bother grabbing the driver if config is valid
    auto lld = ::LLD::getDriver( cfg.HWInit.channel );
    if ( !lld )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Initialize the Low Level Driver
    -------------------------------------------------*/
    lld->enableClock();
    if ( auto sts = lld->configure( cfg ); sts != Chimera::Status::OK )
    {
      return sts;
    }

    /*-------------------------------------------------
    Register the ISR thread handlers
    -------------------------------------------------*/
    size_t lldResourceIndex = ::LLD::getResourceIndex( cfg.HWInit.channel );

    // for ( auto isr_idx = 0; isr_idx < ::LLD::NUM_CAN_IRQ_HANDLERS; isr_idx++ )
    // {
    //   if ( s_user_isr_thread_func[ lldResourceIndex ][ isr_idx ] )
    //   {
    //     Chimera::Thread::Thread thread;
    //     thread.initialize( s_user_isr_thread_func[ lldResourceIndex ][ isr_idx ], nullptr,
    //                        Chimera::Thread::TaskPriority::LEVEL_5, STACK_BYTES( 250 ), nullptr );
    //     thread.start();
    //     s_user_isr_handle[ lldResourceIndex ][ isr_idx ] = thread.native_handle();
    //   }
    // }

    /*-------------------------------------------------
    Initialize the ISR events to listen to
    -------------------------------------------------*/
    lld->enableISRSignal( Chimera::CAN::InterruptType::TX_ISR );
    lld->enableISRSignal( Chimera::CAN::InterruptType::RX_ISR );

    /*-------------------------------------------------
    Update the HLD configuration settings
    -------------------------------------------------*/
    mConfig = cfg;

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::close()
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------
    Disable all ISR signals
    -------------------------------------------------*/
    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );
    for ( auto isr = 0; isr < static_cast<size_t>( InterruptType::NUM_OPTIONS ); isr++ )
    {
      lld->disableISRSignal( static_cast<InterruptType>( isr ) );
    }

    /*-------------------------------------------------
    Ensure the software and hardware buffers are empty
    -------------------------------------------------*/
    lld->flushTX();
    lld->flushRX();

    /*-------------------------------------------------
    Finally, power off the hardware
    -------------------------------------------------*/
    lld->disableClock();

    return Chimera::Status::OK;
  }


  Chimera::CAN::CANStatus Driver::getStatus()
  {
    return Chimera::CAN::CANStatus();
  }


  Chimera::Status_t Driver::send( const Chimera::CAN::BasicFrame &frame )
  {
    /*-------------------------------------------------
    Grab the low level driver
    -------------------------------------------------*/
    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );

    /*-------------------------------------------------
    Ensure transmit ISR is enabled
    -------------------------------------------------*/
    lld->enableISRSignal( Chimera::CAN::InterruptType::TX_ISR );
    lld->enableISRSignal( Chimera::CAN::InterruptType::RX_ISR );
    lld->enableISRSignal( Chimera::CAN::InterruptType::ERR_ISR );

    /*-------------------------------------------------
    Send off the message
    -------------------------------------------------*/
    return lld->send( frame );
  }


  Chimera::Status_t Driver::receive( Chimera::CAN::BasicFrame &frame )
  {
    return ::LLD::getDriver( mConfig.HWInit.channel )->receive( frame );
  }


  Chimera::Status_t Driver::filter( const Chimera::CAN::Filter *const list, const size_t size )
  {
    /*-------------------------------------------------
    Limit the filters to 32-bit mask mode for now and
    expand to more when needed. This still gives around
    14 filters, which is a lot.
    -------------------------------------------------*/
    constexpr size_t hwFilterLength = ::LLD::NUM_CAN_MAX_32BIT_MASK_FILTERS;

    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( !list || !size || ( size > hwFilterLength ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Allocate the appropriate amount of memory on the
    stack and initialize it.
    ------------------------------------------------*/
    ::LLD::MessageFilter hwFilters[ hwFilterLength ];
    for ( auto x = 0; x < hwFilterLength; x++ )
    {
      hwFilters[ x ].clear();
    }

    /*-------------------------------------------------
    Convert the high level filter into the internal
    representation of a hardware filter.
    ------------------------------------------------*/
    ::LLD::Mailbox lastBox = ::LLD::Mailbox::RX_MAILBOX_1;

    for ( auto x = 0; x < size; x++ )
    {
      /*-------------------------------------------------
      Assign basic metrics directly
      -------------------------------------------------*/
      hwFilters[ x ].active     = true;
      hwFilters[ x ].valid      = true;
      hwFilters[ x ].fifoBank   = lastBox;
      hwFilters[ x ].filterType = Thor::CAN::FilterType::MODE_32BIT_MASK;
      hwFilters[ x ].frameType  = Chimera::CAN::FrameType::DATA;
      hwFilters[ x ].identifier = list[ x ].id;
      hwFilters[ x ].mask       = list[ x ].mask;

      /*-------------------------------------------------
      Assign the identifier type
      -------------------------------------------------*/
      hwFilters[ x ].idType = Chimera::CAN::IdType::STANDARD;
      if ( list[ x ].extended )
      {
        hwFilters[ x ].idType = Chimera::CAN::IdType::EXTENDED;
      }

      /*-------------------------------------------------
      Swap which mailbox will be assigned next. Helps to
      load balance the hardware FIFOs.
      -------------------------------------------------*/
      if ( lastBox == ::LLD::Mailbox::RX_MAILBOX_1 )
      {
        lastBox = ::LLD::Mailbox::RX_MAILBOX_2;
      }
      else
      {
        lastBox = ::LLD::Mailbox::RX_MAILBOX_1;
      }
    }

    return ::LLD::getDriver( mConfig.HWInit.channel )->applyFilters( hwFilters, hwFilterLength );
  }


  Chimera::Status_t Driver::flush( Chimera::CAN::BufferType buffer )
  {
    using namespace Chimera::CAN;
    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );

    switch ( buffer )
    {
      case BufferType::RX:
        lld->flushRX();
        break;

      case BufferType::TX:
        lld->flushTX();
        break;

      default:
        return Chimera::Status::INVAL_FUNC_PARAM;
        break;
    }

    return Chimera::Status::OK;
  }


  size_t Driver::available()
  {
    return ::LLD::getDriver( mConfig.HWInit.channel )->available();
  }


  /*------------------------------------------------
  Async IO Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Thread::BinarySemaphore &notifier,
                                   const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  /*------------------------------------------------
  Listener Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  /*-------------------------------------------------
  ISR Event Handlers
  -------------------------------------------------*/
  void Driver::ProcessISREvent_TX()
  {
    using namespace Chimera::CAN;
    using namespace Chimera::Thread;

    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );
    auto box = ::LLD::Mailbox::UNKNOWN;

    /*-------------------------------------------------
    Get the context of the last ISR
    -------------------------------------------------*/
    auto context = lld->getISRContext( InterruptType::RX_ISR );

    /*-------------------------------------------------
    Invoke any user callback
    -------------------------------------------------*/
    // TX?
    // TX Error?

    /*-------------------------------------------------
    Signal internal semaphores for events
    -------------------------------------------------*/
    // Any events to signal?
  }


  void Driver::ProcessISREvent_RX()
  {
    using namespace Chimera::CAN;
    using namespace Chimera::Thread;

    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );

    /*-------------------------------------------------
    Get the context of the last ISR
    -------------------------------------------------*/
    auto context = lld->getISRContext( InterruptType::RX_ISR );

    /*-------------------------------------------------
    Invoke user call back on the RX event
    -------------------------------------------------*/
    // todo
  }


  void Driver::ProcessISREvent_Error()
  {
    using namespace Chimera::CAN;
    using namespace Chimera::Thread;

    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );

    /*-------------------------------------------------
    Get the context of the last ISR
    -------------------------------------------------*/
    auto context = lld->getISRContext( InterruptType::ERR_ISR );

    /*-------------------------------------------------
    Invoke user call back on the RX event
    -------------------------------------------------*/
    // todo
  }


  void Driver::ProcessISREvent_StatusChange()
  {
    using namespace Chimera::CAN;
    using namespace Chimera::Thread;

    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );

    /*-------------------------------------------------
    Get the context of the last ISR
    -------------------------------------------------*/
    auto context = lld->getISRContext( InterruptType::STS_ISR );

    /*-------------------------------------------------
    Invoke user call back on the RX event
    -------------------------------------------------*/
    // todo
  }

}    // namespace Thor::CAN

/*-------------------------------------------------------------------------------
High Priority Threads:
These handle ISR events with the context of a full scheduler
-------------------------------------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
static void CAN1ISR_TXHandler( void *argument )
{
  using namespace Chimera::CAN;

  /*-------------------------------------------------
  Get a reference to the semaphore that will be given
  to once an event occurs.
  -------------------------------------------------*/
  auto sig = ::LLD::getDriver( Channel::CAN0 )->getISRSignal( InterruptType::TX_ISR );

  /*-------------------------------------------------
  Acquire the signal in a blocking manner. Once the
  ISR has signaled the event, process it here.
  -------------------------------------------------*/
  while ( 1 )
  {
    sig->acquire();
    hld_driver[::LLD::CAN1_RESOURCE_INDEX ].ProcessISREvent_TX();
  }
}


static void CAN1ISR_RXHandler( void *argument )
{
  using namespace Chimera::CAN;

  /*-------------------------------------------------
  Get a reference to the semaphore that will be given
  to once an event occurs.
  -------------------------------------------------*/
  auto sig = ::LLD::getDriver( Channel::CAN0 )->getISRSignal( InterruptType::RX_ISR );

  /*-------------------------------------------------
  Acquire the signal in a blocking manner. Once the
  ISR has signaled the event, process it here.
  -------------------------------------------------*/
  while ( 1 )
  {
    sig->acquire();
    hld_driver[::LLD::CAN1_RESOURCE_INDEX ].ProcessISREvent_RX();
  }
}


static void CAN1ISR_StatusChangeHandler( void *argument )
{
  using namespace Chimera::CAN;

  /*-------------------------------------------------
  Get a reference to the semaphore that will be given
  to once an event occurs.
  -------------------------------------------------*/
  auto sig = ::LLD::getDriver( Channel::CAN0 )->getISRSignal( InterruptType::STS_ISR );

  /*-------------------------------------------------
  Acquire the signal in a blocking manner. Once the
  ISR has signaled the event, process it here.
  -------------------------------------------------*/
  while ( 1 )
  {
    sig->acquire();
    hld_driver[::LLD::CAN1_RESOURCE_INDEX ].ProcessISREvent_StatusChange();
  }
}


static void CAN1ISR_ErrHandler( void *argument )
{
  using namespace Chimera::CAN;

  /*-------------------------------------------------
  Get a reference to the semaphore that will be given
  to once an event occurs.
  -------------------------------------------------*/
  auto sig = ::LLD::getDriver( Channel::CAN0 )->getISRSignal( InterruptType::ERR_ISR );

  /*-------------------------------------------------
  Acquire the signal in a blocking manner. Once the
  ISR has signaled the event, process it here.
  -------------------------------------------------*/
  while ( 1 )
  {
    sig->acquire();
    hld_driver[::LLD::CAN1_RESOURCE_INDEX ].ProcessISREvent_StatusChange();
  }
}

#endif /* STM32_CAN1_PERIPH_AVAILABLE */


#endif /* THOR_HLD_CAN */
