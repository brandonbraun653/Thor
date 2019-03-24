#include <Thor/thor.hpp>
#include <Thor/exceptions.hpp>
#include <Thor/system.hpp>


void BasicErrorHandler( std::string err_msg )
{
  /* If you got here, look at the message and trace back to the root problem */
  volatile std::string message = err_msg;

  for ( ;; )
  {
    printf( err_msg.c_str() );
    Thor::delayMilliseconds( 1000 );
  }
}


/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */

#ifdef __cplusplus
extern "C"
{
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  void HardFault_HandlerC( unsigned long *hardfault_args )
  {
#if defined( TARGET_STM32F4 ) || defined( TARGET_STM32F7 )
    /*------------------------------------------------
    Common Registers Between Cortex M4 & M7
    ------------------------------------------------*/
    volatile unsigned long stacked_r0;               // General Purpose Register
    volatile unsigned long stacked_r1;               // General Purpose Register
    volatile unsigned long stacked_r2;               // General Purpose Register
    volatile unsigned long stacked_r3;               // General Purpose Register
    volatile unsigned long stacked_r12;              // General Purpose Register
    volatile unsigned long stacked_lr;               // Link Register: Stores where the program will return to once a
                                                     // subroutine/function/exception is complete
    volatile unsigned long stacked_pc;               // Program Counter: Stores the current program address
    volatile unsigned long stacked_psr;              // Program Status Register:	A set of flags describing the program state
    volatile unsigned long _CFSR = *SCB_REG_CFSR;    // Configurable Fault Status Register
    volatile unsigned long _HFSR = *SCB_REG_HFSR;    // Hard Fault Status Register
    volatile unsigned long _AFSR = *SCB_REG_AFSR;    // Auxiliary Fault Status Register
    volatile unsigned long _BFAR = *SCB_REG_BFAR;    // Bus Fault Address Register
    volatile unsigned long _MMAR = *SCB_REG_MMAR;    // MemManage Fault Address Register

#if defined( TARGET_STM32F4 )
    volatile unsigned long _DFSR = *SCB_REG_DFSR;    // Debug Fault Status Register
#endif

    stacked_r0  = ( ( unsigned long )hardfault_args[ 0 ] );
    stacked_r1  = ( ( unsigned long )hardfault_args[ 1 ] );
    stacked_r2  = ( ( unsigned long )hardfault_args[ 2 ] );
    stacked_r3  = ( ( unsigned long )hardfault_args[ 3 ] );
    stacked_r12 = ( ( unsigned long )hardfault_args[ 4 ] );
    stacked_lr  = ( ( unsigned long )hardfault_args[ 5 ] );
    stacked_pc  = ( ( unsigned long )hardfault_args[ 6 ] );
    stacked_psr = ( ( unsigned long )hardfault_args[ 7 ] );
#endif


    __asm( "BKPT #0\n" );    // Break into the debugger
  }

#pragma GCC diagnostic pop

#ifdef __cplusplus
}
#endif


void HardFault_Handler()
{
  /* Well you broke SOMETHING Jim Bob */
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

  while ( 1 ) {}
}
