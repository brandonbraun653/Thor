/********************************************************************************
 *   File Name:
 *     startup_common_init.cpp
 *
 *   Description:
 *     Provides STM32F4xx generic startup functionality
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstdlib>

extern void SystemInit();
extern int main();
extern void _exit( const int code );

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
      *pDest = *pSource;

    /*------------------------------------------------
    Zero initialize the bss segment
    ------------------------------------------------*/
    for ( pDest = &_sbss; pDest != &_ebss; pDest++ )
      *pDest = 0;
    
    /*------------------------------------------------
    Perform any STM32F4 specific initialization steps
    ------------------------------------------------*/
    SystemInit();

    /*------------------------------------------------
    Takes care of array/ctor/dtor data?
    https://stackoverflow.com/questions/15265295/understanding-the-libc-init-array
    ------------------------------------------------*/
    __libc_init_array();

    /*------------------------------------------------
    System Execution Entry Point
    ------------------------------------------------*/
    volatile int retCode = main();

    /*------------------------------------------------
    Should we ever exit main for some reason, make sure we
    gracefully reset the chip.
    ------------------------------------------------*/
    _exit( retCode );

    /*------------------------------------------------
    Should never get here as _exit() has already reset the chip
    ------------------------------------------------*/
    while ( 1 )
      ;
  }

#if defined( __cplusplus )
}
#endif
