/******************************************************************************
 *  File Name:
 *    cm4_utilities.cpp
 *
 *  Description:
 *    Utilities implementation for the Cortex-M4 devices. For any references that
 *    are tagged in the documentation, it is refering to the ARMv7-M Architecture
 *    Reference Manual.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Thor Includes */
#include <Thor/lld/common/cortex-m4/register.hpp>
#include <Thor/lld/common/cortex-m4/utilities.hpp>


namespace CortexM4
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  uint32_t disableInterrupts()
  {
    /*-------------------------------------------------------------------------
    Read the PRIMASK core register so we can know what state
    interrupts are currently in.
    -------------------------------------------------------------------------*/
    uint32_t primask = 0;
    __asm volatile( "MRS %0, primask" : "=r"( primask )::"memory" );    // Stores the mask
    __asm volatile( "CPSID I" );                                        // Disables interrupts
    return primask;
  }


  void enableInterrupts( const uint32_t mask )
  {
    if ( !mask )
    {
      __asm volatile( "CPSIE I" );
    }
  }


  bool inISR()
  {
    /*-------------------------------------------------------------------------
    If the VECTACTIVE bits are zero, then the processor
    is running in thread mode. Otherwise, it will hold
    the exception number of whatever exception is being
    executed.
    -------------------------------------------------------------------------*/
    return static_cast<bool>( ( *SCB_REG_ICSR ) & ICSR_VECTACTIVE );
  }


  void hardwareReset()
  {
    uint32_t tmp = *SCB_REG_AIRCR;

    /*-------------------------------------------------------------------------
    Use the key to ensure the write will stick (B3.2.6)
    -------------------------------------------------------------------------*/
    tmp &= ~( AIRCR_VECTKEY_Msk );
    tmp |= ( AIRCR_VECTKEY | AIRCR_SYSRESETREQ );

    /*-------------------------------------------------------------------------
    Reset doesn't take place immediately (B1.5.16) so
    idle away until the hardware can handle it.
    -------------------------------------------------------------------------*/
    *SCB_REG_AIRCR = tmp;
    while ( 1 )
    {
      continue;
    }
  }


  bool isDebuggerAttached()
  {
    /*-------------------------------------------------------------------------
    See section C1.6.2. If this bit is set, debugger is attached.
    -------------------------------------------------------------------------*/
    volatile uint32_t val = *SCB_REG_DHCSR;
    return ( val & DHCSR_C_DEBUGEN ) == DHCSR_C_DEBUGEN;
  }

}    // namespace CortexM4
