/********************************************************************************
 *  File Name:
 *    cm4_startup_reset.cpp
 *
 *  Description:
 *    Startup and reset entry point for Cortex-M4 based projects
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#if defined( EMBEDDED )

/* C++ Includes */
#include <cstdlib>

extern void SystemInit();
extern int main();

#if defined( __cplusplus )
extern "C"
{
#endif
  void __libc_init_array();

  /*------------------------------------------------
  Linker script variables
  ------------------------------------------------*/
  extern void *_sidata, *_sdata, *_edata;
  extern void *_sbss, *_ebss;

  /**
   *  Function that is executed whenever a system reset occurs.
   *
   *  @return void
   */
  void __attribute__( ( noreturn ) ) Reset_Handler()
  {
    /*------------------------------------------------
    Copy the data segment from flash into RAM
    ------------------------------------------------*/
    void **pSource, **pDest;
    for ( pSource = &_sidata, pDest = &_sdata; pDest != &_edata; pSource++, pDest++ )
    {
      *pDest = *pSource;
    }

    /*------------------------------------------------
    Zero initialize the bss segment
    ------------------------------------------------*/
    for ( pDest = &_sbss; pDest != &_ebss; pDest++ )
    {
      *pDest = 0;
    }

    /*------------------------------------------------
    Takes care of array/ctor/dtor data
    https://stackoverflow.com/questions/15265295/understanding-the-libc-init-array
    ------------------------------------------------*/
    __libc_init_array();

    /*------------------------------------------------
    Perform any chip specific initialization steps
    ------------------------------------------------*/
    SystemInit();

    /*------------------------------------------------
    System Execution Entry Point
    ------------------------------------------------*/
    int retCode = main();

    /*------------------------------------------------
    Should never get here
    ------------------------------------------------*/
    while ( 1 )
    {
      continue;
    }
  }

#if defined( __cplusplus )
}
#endif

#endif /* EMBEDDED */