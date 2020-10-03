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
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/can>
#include <Chimera/thread>
#include <Chimera/event>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/can>
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

using ThreadHandle = Chimera::Threading::detail::native_thread_handle_type;
using BinarySemphr = Chimera::Threading::BinarySemaphore;
using ThreadFunctn = Chimera::Function::void_func_void_ptr;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = LLD::NUM_CAN_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static size_t s_driver_initialized;                        /**< Tracks the module level initialization state */
static HLD::Driver hld_driver[ NUM_DRIVERS ];              /**< Driver objects */
static HLD::Driver_sPtr hld_shared[ NUM_DRIVERS ];         /**< Shared references to driver objects */
static ThreadHandle s_user_isr_handle[ NUM_DRIVERS ];      /**< Handle to the ISR post processing thread */
static BinarySemphr s_user_isr_signal[ NUM_DRIVERS ];      /**< Lock for each ISR post processing thread */
static ThreadFunctn s_user_isr_thread_func[ NUM_DRIVERS ]; /**< RTOS aware function to execute at end of ISR */

/*-------------------------------------------------------------------------------
Private Function Declarations
-------------------------------------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
static void CAN1ISRPostProcessorThread( void *argument );
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
static void CAN2ISRPostProcessorThread( void *argument );
#endif
#if defined( STM32_CAN3_PERIPH_AVAILABLE )
static void CAN3ISRPostProcessorThread( void *argument );
#endif
#if defined( STM32_CAN4_PERIPH_AVAILABLE )
static void CAN4ISRPostProcessorThread( void *argument );
#endif

namespace Thor::CAN
{
  using namespace Chimera::Threading;

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
    result |= cfg.RXInit.validity;
    result |= cfg.TXInit.validity;

    /*-------------------------------------------------
    The buffers must be pre-allocated as these drivers
    do not use dynamic memory. If dynamic memory is
    desired, it must be allocated externally.
    -------------------------------------------------*/
    result |= ( cfg.HWInit.rxBuffer != nullptr );
    result |= ( cfg.HWInit.txBuffer != nullptr );
    result |= ( cfg.HWInit.rxElements != 0 );
    result |= ( cfg.HWInit.txElements != 0 );

    /*-------------------------------------------------
    Last but not least, does the channel even exist?
    -------------------------------------------------*/
    result |= ( cfg.HWInit.channel < Chimera::CAN::Channel::NUM_OPTIONS );

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
    ::LLD::initialize();

    /*------------------------------------------------
    Initialize ISR post-processing routines
    ------------------------------------------------*/
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::CAN1_RESOURCE_INDEX ] = CAN1ISRPostProcessorThread;
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::CAN2_RESOURCE_INDEX ] = CAN2ISRPostProcessorThread;
#endif
#if defined( STM32_CAN3_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::CAN3_RESOURCE_INDEX ] = CAN3ISRPostProcessorThread;
#endif
#if defined( STM32_CAN4_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::CAN4_RESOURCE_INDEX ] = CAN4ISRPostProcessorThread;
#endif

    /*-------------------------------------------------
    Initialize shared references to drivers
    -------------------------------------------------*/
    for ( size_t x = 0; x < NUM_DRIVERS; x++ )
    {
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_CAN )
      hld_shared[ x ] = ::HLD::Driver_sPtr( new ::HLD::Driver() );
#else
      hld_shared[ x ] = ::HLD::Driver_sPtr( &hld_driver[ x ] );
#endif
    }

    /*-------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
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
    if ( auto idx = ::LLD::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return &hld_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  Driver_sPtr getDriverShared( const Chimera::CAN::Channel channel )
  {
    if ( auto idx = ::LLD::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return hld_shared[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPinTX( nullptr ), mPinRX( nullptr )
  {
    mConfig.clear();
  }


  Driver::~Driver()
  {
  }


  void Driver::postISRProcessing()
  {

  }

  /*------------------------------------------------
  HW Interface
  ------------------------------------------------*/
  Chimera::Status_t open( const Chimera::CAN::DriverConfig &cfg )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if( !validateCfg( cfg ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------

    -------------------------------------------------*/



    return Chimera::Status::OK;
  }


  Chimera::Status_t close()
  {
    return Chimera::Status::OK;
  }


  Chimera::CAN::CANStatus getStatus()
  {
    return Chimera::CAN::CANStatus();
  }


  Chimera::Status_t send( const Chimera::CAN::BasicFrame &frame )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t receive( Chimera::CAN::BasicFrame &frame, const size_t timeout )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t subscribe( const Chimera::CAN::Identifier_t id, Chimera::CAN::FrameCallback_t callback )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t unsubscribe( const Chimera::CAN::Identifier_t id )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t filter( const Chimera::CAN::Filter *const list, const size_t size )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t flush( Chimera::CAN::BufferType buffer )
  {
    return Chimera::Status::OK;
  }


  /*------------------------------------------------
  Async IO Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    if ( event != Chimera::Event::TRIGGER_TRANSFER_COMPLETE )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
    else if ( !awaitTransferComplete.try_acquire_for( timeout ) )
    {
      return Chimera::Status::TIMEOUT;
    }
    else
    {
      return Chimera::Status::OK;
    }
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                                   const size_t timeout )
  {
    auto result = await( event, timeout );

    if ( result == Chimera::Status::OK )
    {
      notifier.release();
    }

    return result;
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

}    // namespace Thor::CAN


#if defined( STM32_CAN1_PERIPH_AVAILABLE )
static void CAN1ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::CAN1_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif

#if defined( STM32_CAN2_PERIPH_AVAILABLE )
static void CAN2ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::CAN2_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif

#if defined( STM32_CAN3_PERIPH_AVAILABLE )
static void CAN3ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::CAN3_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif

#if defined( STM32_CAN4_PERIPH_AVAILABLE )
static void CAN4ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::CAN4_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif

#endif /* THOR_HLD_CAN */
