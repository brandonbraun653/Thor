/******************************************************************************
 *  File Name:
 *    cm4_startup_reset.cpp
 *
 *  Description:
 *    Startup and reset entry point for Cortex-M4 based projects
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#if defined( EMBEDDED )
/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/register.hpp>
#include <cstdlib>

extern void SystemInit();
extern int main();

#if defined( __cplusplus )
extern "C"
{
#endif

  /*---------------------------------------------------------------------------
  Lib Nano Function
  ---------------------------------------------------------------------------*/
  void __libc_init_array();

  /*---------------------------------------------------------------------------
  Linker script variables
  ---------------------------------------------------------------------------*/
  extern void *_sidata, *_sdata, *_edata;
  extern void *_sbss, *_ebss;

  /**
   *  Function that is executed whenever a system reset occurs.
   *  @return void
   */
  void __attribute__( ( noreturn ) ) Reset_Handler()
  {
    /*-------------------------------------------------------------------------
    Cause all imprecise bus fault access errors to become precise.
    -------------------------------------------------------------------------*/
#if defined( WRITE_BUFFERING_DISABLED ) && ( WRITE_BUFFERING_DISABLED == 1 )
    volatile uint32_t actlr = *SCB_REG_ACTLR;
    actlr |= ACTLR_DISDEFWBUF_Msk;
    *SCB_REG_ACTLR = actlr;
#endif /* WRITE_BUFFERING_DISABLED */

    /*-------------------------------------------------------------------------
    Copy the data segment from flash into RAM
    -------------------------------------------------------------------------*/
    void **pSource, **pDest;
    for ( pSource = &_sidata, pDest = &_sdata; pDest != &_edata; pSource++, pDest++ )
    {
      *pDest = *pSource;
    }

    /*-------------------------------------------------------------------------
    Zero initialize the bss segment
    -------------------------------------------------------------------------*/
    for ( pDest = &_sbss; pDest != &_ebss; pDest++ )
    {
      *pDest = 0;
    }

    /*-------------------------------------------------------------------------
    Takes care of array/ctor/dtor data
    https://stackoverflow.com/questions/15265295/understanding-the-libc-init-array

    -------------------------------------------------------------------------*/
    __libc_init_array();

    /*-------------------------------------------------------------------------
    Perform any chip specific initialization steps
    -------------------------------------------------------------------------*/
    SystemInit();

    /*-------------------------------------------------------------------------
    System Execution Entry Point
    -------------------------------------------------------------------------*/
    int UNUSED( retCode ) = main();

    /*-------------------------------------------------------------------------
    Should never get here
    -------------------------------------------------------------------------*/
    while ( 1 )
    {
      continue;
    }
  }

#if defined( __cplusplus )
}
#endif
#endif /* EMBEDDED */
