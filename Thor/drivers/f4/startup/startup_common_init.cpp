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
  extern size_t *_sidata; /* .data Section start from the linker script */
  extern size_t *_sdata;  /* SRAM start location for .data */
  extern size_t *_edata;  /* SRAM end location for .data */
  extern size_t *_sbss;   /* Region start for zero initialized data (.bss) */
  extern size_t *_ebss;   /* Region end for zero initialized data (.bss )*/

  /**
   *  Copies the initial values for variables from .data section into SRAM
   *
   *  @param[in]  from          Where the linker says the .data section begins
   *  @param[in]  region_begin  Start of SRAM for program
   *  @param[in]  region_end    End of SRAM for program
   *  @return void
   */
  static void __attribute__( ( always_inline ) ) __initialize_data( size_t *from, size_t *region_begin, size_t *region_end );

  /**
   *  Zero initializes all data in the .bss section
   *
   *  @param[in]  region_begin  Start of the zero init region
   *  @param[in]  region_end    End of the zero init region
   *  @return void
   */
  static void __attribute__( ( always_inline ) ) __initialize_bss( size_t *region_begin, size_t *region_end );

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
    __initialize_data( _sidata, _sdata, _edata );

    /*------------------------------------------------
    Zero initialize the bss segment
    ------------------------------------------------*/
    __initialize_bss( _sbss, _ebss );

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

  static inline void __initialize_data( size_t *from, size_t *region_begin, size_t *region_end )
  {
    /*------------------------------------------------
    Iterate and copy word by word. It is assumed that 
    the pointers are word aligned.
    ------------------------------------------------*/
    size_t *p = region_begin;
    while ( p < region_end )
    {
      *p++ = *from++;
    }
  }

  static inline void __initialize_bss( size_t *region_begin, size_t *region_end )
  {
    /*------------------------------------------------
    Iterate and clear word by word. It is assumed that 
    the pointers are word aligned.
    ------------------------------------------------*/
    size_t *p = region_begin;
    while ( p < region_end )
    {
      *p++ = 0;
    }
  }

#if defined( __cplusplus )
}
#endif
