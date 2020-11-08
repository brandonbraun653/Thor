/********************************************************************************
 *  File Name:
 *    cm4_utilities.cpp
 *
 *  Description:
 *    Utilities implementation for the Cortex-M4 devices
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/common/cortex-m4/register.hpp>
#include <Thor/lld/common/cortex-m4/utilities.hpp>


namespace CortexM4
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/

  uint32_t disableInterrupts()
  {
    /*------------------------------------------------
    Read the PRIMASK core register so we can know what state
    interrupts are currently in.
    ------------------------------------------------*/
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
    /*-------------------------------------------------
    If the VECTACTIVE bits are zero, then the processor
    is running in thread mode. Otherwise, it will hold
    the exception number of whatever exception is being
    executed.
    -------------------------------------------------*/
    return static_cast<bool>( ( *SCB_REG_ICSR ) & ICSR_VECTACTIVE );
  }

}    // namespace CortexM4
