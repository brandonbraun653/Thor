/********************************************************************************
 *  File Name:
 *    thor_exceptions.cpp
 *
 *  Description:
 *    Thor exception handling implementations
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/cfg>

#if defined( CORTEX_M4 ) && defined( EMBEDDED )

/* Driver Includes */
#include <Thor/lld/common/cortex-m4/register.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   *  Exception handler registered with the isr vector array
   *  @note Pulls the fault stack and calls a C function that interprets it
   *
   *  @return void
   */
  void HardFault_Handler()
  {
    /* Well you broke SOMETHING Jim Bob */

#if defined( __GNUC__ )
    __asm volatile( " movs r0,#4			\n"
                    " movs r1, lr			\n"
                    " tst r0, r1			\n"
                    " beq _MSP				\n"
                    " mrs r0, psp			\n"
                    " b _HALT				\n"
                    "_MSP:					\n"
                    " mrs r0, msp			\n"
                    "_HALT:					\n"
                    " ldr r1,[r0,#20]		\n"
                    " b HardFault_HandlerC	\n"
                    " bkpt #0				\n" );
#endif

    while ( 1 ) {}
  }


#if defined( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif

  /**
   *  This is called from HardFault_Handler() with a pointer to the fault stack
   *  as the parameter. We can then read the values from the stack and place them
   *  into local variables for ease of reading.
   *
   *  @note This function never returns as we break into the debugger.
   *
   *  @param[in]  hardfault_args    Pointer to the fault stack
   *  @return void
   */
  void HardFault_HandlerC( unsigned long *hardfault_args )
  {
    /*------------------------------------------------
    Common Registers Between Cortex M4 & M7
    ------------------------------------------------*/
    volatile unsigned long stacked_r0;            /* General Purpose Register */
    volatile unsigned long stacked_r1;            /* General Purpose Register */
    volatile unsigned long stacked_r2;            /* General Purpose Register */
    volatile unsigned long stacked_r3;            /* General Purpose Register */
    volatile unsigned long stacked_r12;           /* General Purpose Register */
    volatile unsigned long stacked_lr;            /* Link Register: Stores where the program will return to once a */
                                                  /* subroutine/function/exception is complete */
    volatile unsigned long stacked_pc;            /* Program Counter: Stores the current program address */
    volatile unsigned long stacked_psr;           /* Program Status Register:	A set of flags describing the program state */
    volatile unsigned long _CFSR = *SCB_REG_CFSR; /* Configurable Fault Status Register */
    volatile unsigned long _BFSR = *SCB_REG_BFSR; /* Bus fault status register (subset of CFSR) */
    volatile unsigned long _UFSR = *SCB_REG_UFSR; /* Usage fault status register (subset of CFSR) */
    volatile unsigned long _MMSR = *SCB_REG_MMSR; /* MemManage fault status register (subset of CFSR) */
    volatile unsigned long _HFSR = *SCB_REG_HFSR; /* Hard Fault Status Register */
    volatile unsigned long _AFSR = *SCB_REG_AFSR; /* Auxiliary Fault Status Register */
    volatile unsigned long _BFAR = *SCB_REG_BFAR; /* Bus Fault Address Register */
    volatile unsigned long _MMAR = *SCB_REG_MMAR; /* MemManage Fault Address Register */
    volatile unsigned long _DFSR = *SCB_REG_DFSR; /* Debug Fault Status Register */

    stacked_r0  = ( ( unsigned long )hardfault_args[ 0 ] );
    stacked_r1  = ( ( unsigned long )hardfault_args[ 1 ] );
    stacked_r2  = ( ( unsigned long )hardfault_args[ 2 ] );
    stacked_r3  = ( ( unsigned long )hardfault_args[ 3 ] );
    stacked_r12 = ( ( unsigned long )hardfault_args[ 4 ] );
    stacked_lr  = ( ( unsigned long )hardfault_args[ 5 ] );
    stacked_pc  = ( ( unsigned long )hardfault_args[ 6 ] );
    stacked_psr = ( ( unsigned long )hardfault_args[ 7 ] );

    /*-------------------------------------------------------------------------------
    Configurable Fault Status Register
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Bus Fault Status Register
    -------------------------------------------------*/
    /* Bus Fault Address Register has valid contents */
    volatile bool BFAR_VALID = static_cast<bool>( _BFSR & BFSR_BFARVALID );

    /* Imprecise data access has occurred */
    volatile bool BFSR_IMPRECISE = static_cast<bool>( _BFSR & BFSR_IMPRECISERR );

    /* Precise data access has occurred */
    volatile bool BFSR_PRECISE = static_cast<bool>( _BFSR & BFSR_PRECISERR );


    /*-------------------------------------------------
    Usage Fault Status Register
    -------------------------------------------------*/
    /* A divide by zero has occurred */
    volatile bool DIV_ZERO = static_cast<bool>( _UFSR & UFSR_DIVBYZERO );

    /* Unaligned access has occurred */
    volatile bool UNALIGNED_ACCESS = static_cast<bool>( _UFSR & UFSR_UNALIGNED );

    /* Undefined instruction attempted to be executed */
    volatile bool UNDEF_INSTRUCTION = static_cast<bool>( _UFSR & UFSR_UNDEFINSTR );


// TODO Switch behavior based on debug or release
#if defined( __GNUC__ )
    __asm( "BKPT #0\n" );    // Break into the debugger
#endif
  }

#if defined( __GNUC__ )
#pragma GCC diagnostic pop
#endif

#ifdef __cplusplus
}
#endif

#endif /* CORTEX_M4 && EMBEDDED */
