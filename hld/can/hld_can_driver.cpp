/********************************************************************************
 *  File Name:
 *    hld_can_driver.cpp
 *
 *  Description:
 *    CAN driver for Thor
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
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
#include <Thor/lld/interface/inc/can>
#include <Thor/lld/interface/inc/interrupt>

#if defined( THOR_CAN )
/*-----------------------------------------------------------------------------
Aliases
-----------------------------------------------------------------------------*/
namespace HLD = ::Thor::CAN;
namespace LLD = ::Thor::LLD::CAN;

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = LLD::NUM_CAN_PERIPHS;
static constexpr size_t NUM_ISR_SIG = LLD::NUM_CAN_IRQ_HANDLERS;

/*-----------------------------------------------------------------------------
Static Variables
-----------------------------------------------------------------------------*/
static size_t s_driver_initialized;
static HLD::Driver hld_driver[ NUM_DRIVERS ];


namespace Thor::CAN
{
  using namespace Chimera::Thread;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Validates the CAN driver configuration
   *
   * @param cfg     Config to validate
   * @return true   Is valid
   * @return false  Not valid
   */
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


  /**
   * @brief High level handler for CAN instance ISR events
   *
   * @param arg   Unused
   */
  static void CANxISRUserThread( void *arg )
  {
    using namespace Chimera::Thread;

    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Wait for an ISR event to wake this thread
      -----------------------------------------------------------------------*/
      if ( !this_thread::pendTaskMsg( ITCMsg::TSK_MSG_ISR_HANDLER ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Handle every ISR. Don't know which triggered this
      -----------------------------------------------------------------------*/
      for ( size_t index = 0; index < NUM_DRIVERS; index++ )
      {
        hld_driver[ index ].postISRProcessing();
      }
    }
  }


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
    Register the ISR post processor thread
    -------------------------------------------------------------------------*/
    Task userThread;
    TaskConfig cfg;

    cfg.arg        = nullptr;
    cfg.function   = CANxISRUserThread;
    cfg.priority   = Priority::MAXIMUM;
    cfg.stackWords = STACK_BYTES( 512 );
    cfg.type       = TaskInitType::DYNAMIC;
    cfg.name       = "PP_CANx";

    userThread.create( cfg );
    LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_CAN, userThread.start() );

    /*-------------------------------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------------------------------*/
    auto result = ::LLD::initialize();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }


  Chimera::Status_t reset()
  {
    return Chimera::Status::NOT_SUPPORTED;
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


  Chimera::Status_t Driver::open( const Chimera::CAN::DriverConfig &cfg )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !validateCfg( cfg ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    auto lld = ::LLD::getDriver( cfg.HWInit.channel );
    if ( !lld )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------------------------------*/
    lld->enableClock();
    if ( auto sts = lld->configure( cfg ); sts != Chimera::Status::OK )
    {
      return sts;
    }
    else
    {
      mConfig = cfg;
    }

    /*-------------------------------------------------------------------------
    Initialize the ISR events to listen to
    -------------------------------------------------------------------------*/
    lld->enableISRSignal( Chimera::CAN::InterruptType::TX_ISR );
    lld->enableISRSignal( Chimera::CAN::InterruptType::RX_ISR );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::close()
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------------------------------
    Turn off all ISRs to prevent interruption
    -------------------------------------------------------------------------*/
    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );
    for ( size_t isr = 0; isr < static_cast<size_t>( InterruptType::NUM_OPTIONS ); isr++ )
    {
      lld->disableISRSignal( static_cast<InterruptType>( isr ) );
    }

    /*-------------------------------------------------------------------------
    Clear resources and power off the module
    -------------------------------------------------------------------------*/
    lld->flushTX();
    lld->flushRX();
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
    Ensure we are listening to events, then enqueue TX
    -------------------------------------------------*/
    auto lld = ::LLD::getDriver( mConfig.HWInit.channel );
    lld->enableISRSignal( Chimera::CAN::InterruptType::TX_ISR );
    lld->enableISRSignal( Chimera::CAN::InterruptType::RX_ISR );
    lld->enableISRSignal( Chimera::CAN::InterruptType::ERR_ISR );
    return lld->send( frame );
  }


  Chimera::Status_t Driver::receive( Chimera::CAN::BasicFrame &frame )
  {
    return ::LLD::getDriver( mConfig.HWInit.channel )->receive( frame );
  }


  Chimera::Status_t Driver::filter( const Chimera::CAN::Filter *const list, const size_t size )
  {
    /*-------------------------------------------------------------------------
    Limit the filters to 32-bit mask mode for now and expand to more when
    needed. This still gives around 14 filters, which is a lot.
    -------------------------------------------------------------------------*/
    constexpr size_t hwFilterLength = ::LLD::NUM_CAN_MAX_32BIT_MASK_FILTERS;

    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if ( !list || !size || ( size > hwFilterLength ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Allocate the appropriate amount of memory on the stack and initialize it.
    Use memset instead of the clear() method to save memory. Doesn't matter in
    this case that they have special init values.
    -------------------------------------------------------------------------*/
    ::LLD::MessageFilter hwFilters[ hwFilterLength ];
    memset( hwFilters, 0, ARRAY_BYTES( hwFilters ) );

    /*-------------------------------------------------------------------------
    Convert the high level filter into a hardware filter
    -------------------------------------------------------------------------*/
    ::LLD::Mailbox lastBox = ::LLD::Mailbox::RX_MAILBOX_1;

    for ( size_t x = 0; x < size; x++ )
    {
      /*-----------------------------------------------------------------------
      Assign basic metrics directly
      -----------------------------------------------------------------------*/
      hwFilters[ x ].active     = true;
      hwFilters[ x ].valid      = true;
      hwFilters[ x ].fifoBank   = lastBox;
      hwFilters[ x ].filterType = Thor::CAN::FilterType::MODE_32BIT_MASK;
      hwFilters[ x ].frameType  = Chimera::CAN::FrameType::DATA;
      hwFilters[ x ].identifier = list[ x ].id;
      hwFilters[ x ].mask       = list[ x ].mask;

      /*-----------------------------------------------------------------------
      Assign the identifier type
      -----------------------------------------------------------------------*/
      hwFilters[ x ].idType = Chimera::CAN::IdType::STANDARD;
      if ( list[ x ].extended )
      {
        hwFilters[ x ].idType = Chimera::CAN::IdType::EXTENDED;
      }

      /*-----------------------------------------------------------------------
      Swap which mailbox will be assigned next. Helps to load balance HW FIFOs.
      -----------------------------------------------------------------------*/
      if ( lastBox == ::LLD::Mailbox::RX_MAILBOX_1 )
      {
        lastBox = ::LLD::Mailbox::RX_MAILBOX_2;
      }
      else
      {
        lastBox = ::LLD::Mailbox::RX_MAILBOX_1;
      }
    }

    /*-------------------------------------------------------------------------
    Apply the filter configuration
    -------------------------------------------------------------------------*/
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


  void Driver::postISRProcessing()
  {

  }


  void Driver::ProcessISREvent_TX()
  {
  }


  void Driver::ProcessISREvent_RX()
  {
  }


  void Driver::ProcessISREvent_Error()
  {
  }


  void Driver::ProcessISREvent_StatusChange()
  {
  }

}    // namespace Thor::CAN

#endif /* THOR_HLD_CAN */
