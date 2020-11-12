/********************************************************************************
 *  File Name:
 *    hw_exti_driver.cpp
 *
 *  Description:
 *    EXTI driver for the STM32L4 series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Aurora Includes */
#include <Aurora/constants>

/* Chimera Includes */
#include <Chimera/exti>
#include <Chimera/function>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/common/cortex-m4/interrupts.hpp>
#include <Thor/lld/interface/exti/exti_prv_data.hpp>
#include <Thor/lld/interface/exti/exti_intf.hpp>
#include <Thor/lld/stm32l4x/exti/hw_exti_prj.hpp>
#include <Thor/lld/stm32l4x/exti/hw_exti_types.hpp>

// GPIO: get exti line for config?


/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
/* clang-format off */
#define LISTENER_OFFSET( X )    ( ( X <= 31 ) ? 0 : 32 )
#define LISTENER_BIT_SET( X )   ( 1u << ( static_cast<Reg32_t>( X ) - LISTENER_OFFSET( X ) ) )

#define IMR_PTR( listenerId )   ( ( listenerId <= 31 ) ? &EXTI1_PERIPH->IMR1 : &EXTI1_PERIPH->IMR2 )
#define EMR_PTR( listenerId )   ( ( listenerId <= 31 ) ? &EXTI1_PERIPH->EMR1 : &EXTI1_PERIPH->EMR2 )
#define RTSR_PTR( listenerId )  ( ( listenerId <= 31 ) ? &EXTI1_PERIPH->RTSR1 : &EXTI1_PERIPH->RTSR2 )
#define FTSR_PTR( listenerId )  ( ( listenerId <= 31 ) ? &EXTI1_PERIPH->FTSR1 : &EXTI1_PERIPH->FTSR2 )
#define PR_PTR( listenerId )    ( ( listenerId <= 31 ) ? &EXTI1_PERIPH->PR1 : &EXTI1_PERIPH->PR2 )
#define SWIER_PTR( listenerId ) ( ( listenerId <= 31 ) ? &EXTI1_PERIPH->SWIER1 : &EXTI1_PERIPH->SWIER2 )

namespace Thor::LLD::EXTI
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static size_t s_driver_initialized;
  static Chimera::Threading::RecursiveMutex s_mtx;
  static Chimera::Function::vGeneric s_Callbacks[ NUM_EXTI_LINES ];


  /*-------------------------------------------------------------------------------
  Static Function Declaration
  -------------------------------------------------------------------------------*/
  static void IRQHandlerLine0_15( Chimera::EXTI::EventLine_t listener );


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t open()
  {
    /*------------------------------------------------
    Prevent multiple initializations (need reset first)
    ------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*-------------------------------------------------
    Reset the cache and disable the hardware interrupts
    -------------------------------------------------*/
    auto result = Chimera::Status::OK;
    for( Chimera::EXTI::EventLine_t x = 0; x < NUM_EXTI_LINES; x++ )
    {
      result |= detach( x );
    }

    /*-------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }


  Chimera::Status_t close()
  {
    s_mtx.lock();

    /*-------------------------------------------------
    Reset all the listeners
    -------------------------------------------------*/
    auto result = Chimera::Status::OK;
    for( Chimera::EXTI::EventLine_t x = 0; x < NUM_EXTI_LINES; x++ )
    {
      auto irq = static_cast<IRQn_Type>( Config::lineConfig[ x ].NVIC_IRQNumber );
      Thor::LLD::IT::disableIRQ( irq );

      result |= detach( x );
    }

    /*-------------------------------------------------
    Unlock the init sequence and exit
    -------------------------------------------------*/
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    s_mtx.unlock();
    return result;
  }


  Chimera::Status_t attach( const Chimera::EXTI::EventLine_t listener, const Chimera::EXTI::EdgeTrigger edge, Chimera::Function::vGeneric callback )
  {
    using namespace Chimera::EXTI;

    /*-------------------------------------------------
    Argument Checks
    -------------------------------------------------*/
    if( !( listener < NUM_EXTI_LINES ) || !( edge < EdgeTrigger::NUM_OPTIONS ) || !callback )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Support checks
    -------------------------------------------------*/
    if( !Config::lineConfig[ listener ].supported )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Last check. Is this already configured?
    -------------------------------------------------*/
    if( s_Callbacks[ listener ] )
    {
      return Chimera::Status::NOT_AVAILABLE;
    }

    /*-------------------------------------------------
    Otherwise, configure the interrupt
    -------------------------------------------------*/
    s_mtx.lock();

    // Configure interrupt mask register: Masked (0) == disabled, Un-Masked (1) == enabled
    *IMR_PTR(listener) |= LISTENER_BIT_SET( listener );

    // Configure the edge selection
    switch( edge )
    {
      case EdgeTrigger::RISING_EDGE:
        *RTSR_PTR( listener ) |= LISTENER_BIT_SET( listener );
        *FTSR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );
        break;

      case EdgeTrigger::FALLING_EDGE:
        *RTSR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );
        *FTSR_PTR( listener ) |= LISTENER_BIT_SET( listener );
        break;

      case EdgeTrigger::BOTH_EDGE:
        *RTSR_PTR( listener ) |= LISTENER_BIT_SET( listener );
        *FTSR_PTR( listener ) |= LISTENER_BIT_SET( listener );
        break;

      default:
        *RTSR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );
        *FTSR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );
        break;
    };

    // Cache the callback for later use
    s_Callbacks[ listener ] = callback;

    // Enable the NVIC controller's interrupt
    auto irq = static_cast<IRQn_Type>( Config::lineConfig[ listener ].NVIC_IRQNumber );

    Thor::LLD::IT::setPriority( irq, 5, 0 );
    Thor::LLD::IT::enableIRQ( irq );

    s_mtx.unlock();
    return Chimera::Status::OK;
  }


  Chimera::Status_t detach( const Chimera::EXTI::EventLine_t listener )
  {
    if( !( listener < NUM_EXTI_LINES ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    s_mtx.lock();

    /*-------------------------------------------------
    Deconfigure the interrupt signals. Rather than be
    clever, just disable everything associated with the
    listener in question.
    -------------------------------------------------*/
    *IMR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );
    *EMR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );
    *RTSR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );
    *FTSR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );

    // Clear any possible pending events
    *PR_PTR( listener ) |= LISTENER_BIT_SET( listener );

    // Don't disable the NVIC just yet cause other signals share interrupt lines

    /*-------------------------------------------------
    Remove the callback from the cache
    -------------------------------------------------*/
    s_Callbacks[ listener ] = Chimera::Function::vGeneric();

    s_mtx.unlock();
    return Chimera::Status::OK;
  }


  Chimera::Status_t trigger( const Chimera::EXTI::EventLine_t listener )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if( !( listener < NUM_EXTI_LINES ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    If the SWIER register has already been set but the
    PR indicates no event, the hardware has gotten into
    a weird state. Setting the appropriate bit in PR
    (to auto-clear SWIER) should allow SWIER to become
    effective again.
    -------------------------------------------------*/
    const auto listener_bit = LISTENER_BIT_SET( listener );
    const auto swier_set = ( *SWIER_PTR( listener ) & listener_bit );
    const auto pr_set = ( *PR_PTR( listener ) & listener_bit );

    if( swier_set && !pr_set )
    {
      *PR_PTR( listener ) |= listener_bit;
    }

    /*-------------------------------------------------
    Now hardware should be in a good state to handle
    the software interrupt request.
    -------------------------------------------------*/
    *SWIER_PTR( listener ) |= LISTENER_BIT_SET( listener );
    return Chimera::Status::OK;
  }


  Chimera::Status_t disable( const Chimera::EXTI::EventLine_t listener )
  {
    if( !( listener < NUM_EXTI_LINES ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    *IMR_PTR( listener ) &= ~LISTENER_BIT_SET( listener );
    return Chimera::Status::OK;
  }


  Chimera::Status_t enable( const Chimera::EXTI::EventLine_t listener )
  {
    if( !( listener < NUM_EXTI_LINES ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    *IMR_PTR( listener ) |= LISTENER_BIT_SET( listener );
    return Chimera::Status::OK;
  }


  Chimera::EXTI::EventLine_t numInterruptLines()
  {
    return static_cast<Chimera::EXTI::EventLine_t>( NUM_EXTI_LINES );
  }


  /*-------------------------------------------------------------------------------
  Static Function Definitions
  -------------------------------------------------------------------------------*/
  static void IRQHandlerLine0_15( Chimera::EXTI::EventLine_t listener )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if( !( listener < 16 ) )
    {
      return;
    }

    /*-------------------------------------------------
    Acknowledge the event, then invoke the callback
    -------------------------------------------------*/
    if( *PR_PTR( listener ) & LISTENER_BIT_SET( listener ) )
    {
      *PR_PTR( listener ) |= LISTENER_BIT_SET( listener );

      if( s_Callbacks[ listener ])
      {
        s_Callbacks[ listener ]( reinterpret_cast<void*>( &listener ) );
      }
    }
  }
}  // namespace Thor::LLD::EXTI


#if defined( __cplusplus )
extern "C"
{
#endif

  /**
   *  Handle GPIO external interrupts from Pin-0 on all ports. Only
   *  a single pin can be configured to triger this ISR.
   */
  void EXTI0_IRQHandler()
  {
    Thor::LLD::EXTI::IRQHandlerLine0_15( 0 );
  }


  /**
   *  Handle GPIO external interrupts from Pin-1 on all ports. Only
   *  a single pin can be configured to triger this ISR.
   */
  void EXTI1_IRQHandler()
  {
    Thor::LLD::EXTI::IRQHandlerLine0_15( 1 );
  }


  /**
   *  Handle GPIO external interrupts from Pin-2 on all ports. Only
   *  a single pin can be configured to triger this ISR.
   */
  void EXTI2_IRQHandler()
  {
    Thor::LLD::EXTI::IRQHandlerLine0_15( 2 );
  }


  /**
   *  Handle GPIO external interrupts from Pin-3 on all ports. Only
   *  a single pin can be configured to triger this ISR.
   */
  void EXTI3_IRQHandler()
  {
    Thor::LLD::EXTI::IRQHandlerLine0_15( 3 );
  }


  /**
   *  Handle GPIO external interrupts from Pin-4 on all ports. Only
   *  a single pin can be configured to triger this ISR.
   */
  void EXTI4_IRQHandler()
  {
    Thor::LLD::EXTI::IRQHandlerLine0_15( 4 );
  }


  /**
   *  Handle GPIO external interrupts from Pins 5-9 on all ports.
   *  Multiple pins can be configured to triger this ISR.
   */
  void EXTI9_5_IRQHandler()
  {
    for( auto x = 5; x < 10; x++ )
    {
      Thor::LLD::EXTI::IRQHandlerLine0_15( x );
    }
  }


  /**
   *  Handle GPIO external interrupts from Pins 10-15 on all ports.
   *  Multiple pins can be configured to triger this ISR.
   */
  void EXTI15_10_IRQHandler()
  {
    for( auto x = 10; x < 16; x++ )
    {
      Thor::LLD::EXTI::IRQHandlerLine0_15( x );
    }
  }

#if defined( __cplusplus )
}
#endif