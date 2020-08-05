/********************************************************************************
 *  File Name:
 *    sys_common_init.cpp
 *
 *  Description:
 *    System initialization common to all STM32L4xxxx chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/rcc/rcc_detail.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>

/* ARM Includes: Must come last so the Thor Includes can configure various macros */
#include <Thor/lld/common/cmsis/core/include/core_cm4.h>


#if defined( EMBEDDED ) && defined( THOR_LLD_RCC )

void SystemInit()
{
  using namespace Thor::LLD::RCC;

  /*------------------------------------------------
  Clear any other clock configuration data
  ------------------------------------------------*/
  RCC1_PERIPH->CR &= ~( CR_MSION | CR_HSEON | CR_CSSON | CR_PLLON );
  RCC1_PERIPH->CFGR    = 0;
  RCC1_PERIPH->PLLCFGR = 0x00001000;
  RCC1_PERIPH->CIER    = 0;
  RCC1_PERIPH->CR &= ~CR_HSEBYP;    // Can only be written after HSEON is cleared

  /*------------------------------------------------
  Switch the system clock to be driven from the HSI16 bus. While
  techinally we just turned it on above, the hardware will override
  us on exiting Standby or Shutdown mode to use the MSI clock instead.

  This leads to a condition where the first power up will have the
  correct clock frequency, but toggling the nRST pin ends up causing
  the MSI clock to be selected.
  ------------------------------------------------*/
  auto rcc = getCoreClock();
  rcc->enableClock( Chimera::Clock::Bus::HSI16 );
  rcc->setCoreClockSource( Chimera::Clock::Bus::HSI16 );

  /*------------------------------------------------
  Update the external clock variable so FreeRTOS (if used)
  can know the startup frequency.
  ------------------------------------------------*/
  SystemCoreClock = rcc->getClockFrequency( Chimera::Clock::Bus::SYSCLK );

  /*------------------------------------------------
  Default initialize the System Control Block
  ------------------------------------------------*/
#if ( __FPU_PRESENT == 1 ) && ( __FPU_USED == 1 )
  SCB->CPACR |= ( ( 3UL << 10 * 2 ) | ( 3UL << 11 * 2 ) ); /* set CP10 and CP11 Full Access */
#endif

  /* Configure the Vector Table location add offset address */
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = Thor::System::MemoryMap::FLASH_BASE_ADDR |
              Thor::System::MemoryMap::VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}

#elif defined( _SIM )

void SystemInit()
{
  // Do something with this later
}

#else

void SystemInit()
{
  // Needed for compilation
}

#endif
