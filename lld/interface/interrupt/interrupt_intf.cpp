/********************************************************************************
 *  File Name:
 *    interrupt_intf.cpp
 *
 *  Description:
 *    Interface layer implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/interrupt>
#include <Chimera/serial>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/sys>

#if defined( CORTEX_M4 )
#include <Thor/lld/common/cortex-m4/utilities.hpp>
#endif

namespace Thor::LLD::INT
{
  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct HandlerRegistry
  {
    Chimera::Peripheral::Type type;
    Chimera::Interrupt::SignalCallback *const array;
    const size_t elements;
  };


  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static Chimera::System::InterruptMask s_interrupt_state;

  /*-------------------------------------------------
  User-Space ISR Handler Thread Identifiers
  -------------------------------------------------*/
  static Chimera::Thread::TaskId UserTaskId[ ARRAY_COUNT( SYS::AvailablePeriphs ) ];

  /*-------------------------------------------------
  Peripheral Callback Storage
  -------------------------------------------------*/
#if defined( THOR_LLD_USART )
  static Chimera::Interrupt::SignalCallback USARTHandlers[ Chimera::Serial::SIG_NUM_OPTIONS ];
#endif

  /**
   *  Registry for all available peripheral callbacks
   */
  static const HandlerRegistry SignalHandlers[] = {

#if defined( THOR_LLD_USART )
    { Chimera::Peripheral::Type::PERIPH_USART, USARTHandlers, ARRAY_COUNT( USARTHandlers ) },
#endif
  };


  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Looks up the indices associated with a peripheral interrupt cache
   *
   *  @param[in]  type      The peripheral type to look up
   *  @param[in]  signal    Which signal number the peripheral should support
   *  @param[out] index     Peripheral resource index, if found
   *  @return Chimera::Status_t
   */
  static Chimera::Status_t lookup_handler_indices( const Chimera::Peripheral::Type type,
                                                   const Chimera::Interrupt::Signal_t signal, size_t &index )
  {
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( ( type >= Type::NUM_OPTIONS ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /* Ensure the device has this kind of peripheral */
    size_t resourceIndex = INVALID_RESOURCE_INDEX;
    for ( size_t x = 0; x < ARRAY_COUNT( SignalHandlers ); x++ )
    {
      if ( SignalHandlers[ x ].type == type )
      {
        resourceIndex = x;
        break;
      }
    }

    if ( resourceIndex == INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /* Ensure the device has this ISR signal */
    if ( signal >= SignalHandlers[ resourceIndex ].elements )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Type/signal is supported
    -------------------------------------------------*/
    index = resourceIndex;
    return Chimera::Status::OK;
  }


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return reset();
  }


  Chimera::Status_t reset()
  {
    using namespace Chimera::Interrupt;

    /*-------------------------------------------------
    Unregister each item in the handler list
    -------------------------------------------------*/
    for ( size_t x = 0; x < ARRAY_COUNT( SignalHandlers ); x++ )
    {
      for ( size_t y = 0; y < SignalHandlers[ x ].elements; y++ )
      {
        SignalHandlers[ x ].array[ y ].data         = nullptr;
        SignalHandlers[ x ].array[ y ].size         = 0;
        SignalHandlers[ x ].array[ y ].signal       = 0;
        SignalHandlers[ x ].array[ y ].isrCallback  = {};
        SignalHandlers[ x ].array[ y ].userCallback = {};
      }
    }

    /*-------------------------------------------------
    Reset the thread identifiers
    -------------------------------------------------*/
    for ( size_t x = 0; x < ARRAY_COUNT( UserTaskId ); x++ )
    {
      UserTaskId[ x ] = Chimera::Thread::THREAD_ID_INVALID;
    }

    /*-------------------------------------------------
    Reset the interrupt mask counter
    -------------------------------------------------*/
    s_interrupt_state.count = 0;
    s_interrupt_state.mask  = 0;

    return Chimera::Status::OK;
  }


  Chimera::Status_t registerISRHandler( const Chimera::Peripheral::Type type, const Chimera::Interrupt::Signal_t signal,
                                        const Chimera::Interrupt::SignalCallback &callback )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    size_t resourceIndex = 0;
    if ( auto result = lookup_handler_indices( type, signal, resourceIndex ); result != Chimera::Status::OK )
    {
      return result;
    }

    /*-------------------------------------------------
    Assign the data
    -------------------------------------------------*/
    SignalHandlers[ resourceIndex ].array[ signal ] = callback;
    return Chimera::Status::OK;
  }


  const Chimera::Interrupt::SignalCallback *const getISRHandler( const Chimera::Peripheral::Type type,
                                                                 const Chimera::Interrupt::Signal_t signal )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    size_t resourceIndex = 0;
    if ( auto result = lookup_handler_indices( type, signal, resourceIndex ); result != Chimera::Status::OK )
    {
      return nullptr;
    }

    /*-------------------------------------------------
    Return the registered data
    -------------------------------------------------*/
    return &SignalHandlers[ resourceIndex ].array[ signal ];
  }


  void setUserTaskId( const Chimera::Peripheral::Type type, const Chimera::Thread::TaskId id )
  {
    static_assert( ARRAY_COUNT( UserTaskId ) == ARRAY_COUNT( SYS::AvailablePeriphs ) );

    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    size_t resourceIndex = SYS::getResourceIndex( type );
    if ( resourceIndex == INVALID_RESOURCE_INDEX )
    {
      return;
    }

    /*-------------------------------------------------
    Assign the data
    -------------------------------------------------*/
    UserTaskId[ resourceIndex ] = id;
  }


  Chimera::Thread::TaskId getUserTaskId( const Chimera::Peripheral::Type type )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    size_t resourceIndex = SYS::getResourceIndex( type );
    if ( resourceIndex == INVALID_RESOURCE_INDEX )
    {
      return Chimera::Thread::THREAD_ID_INVALID;
    }

    /*-------------------------------------------------
    Return the data
    -------------------------------------------------*/
    return UserTaskId[ resourceIndex ];
  }


  Chimera::System::InterruptMask disableInterrupts()
  {
    /*-------------------------------------------------
    Check the reference counter to see if this is the
    first call to this function.
    -------------------------------------------------*/
    if ( !s_interrupt_state.count )
    {
#if defined( CORTEX_M4 )
      s_interrupt_state.mask = CortexM4::disableInterrupts();
#endif
    }

    /*-------------------------------------------------
    Increment the reference counter for nested calls
    -------------------------------------------------*/
    s_interrupt_state.count += 1;
    return s_interrupt_state;
  }

  void enableInterrupts( Chimera::System::InterruptMask &interruptMask )
  {
    /*-------------------------------------------------
    Prevent underflow on too many calls
    -------------------------------------------------*/
    if ( s_interrupt_state.count > 0 )
    {
      s_interrupt_state.count -= 1;
    }

    /*-------------------------------------------------
    Proper number of calls to unwind the call chain has
    been made. Safe to unlock ISRs now.
    -------------------------------------------------*/
    if ( s_interrupt_state.count == 0 )
    {
#if defined( CORTEX_M4 )
      CortexM4::enableInterrupts( interruptMask.mask );
#endif
    }
  }

}    // namespace Thor::LLD::INT
