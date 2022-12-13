/******************************************************************************
 *  File Name:
 *     sys_common_init.cpp
 *
 *  Description:
 *     Provides STM32F4xx generic initialization functionality
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/sys>
#include <Thor/lld/interface/inc/power>

/* ARM Includes: Must come last so the Thor Includes can configure various macros */
#include <Thor/lld/common/cmsis/core/include/core_cm4.h>
#include <Thor/lld/common/cortex-m4/system_time.hpp>


void SystemInit()
{
  using namespace Thor::LLD::RCC;

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
  SCB->VTOR = Thor::System::MemoryMap::FLASH_RGN_START_ADDR |
              Thor::System::MemoryMap::VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

  // /* Reset the RCC clock configuration to the default reset state ------------*/
  // /* Set HSION bit */
  // RCC1_PERIPH->CR |= ( uint32_t )0x00000001;

  // /* Reset CFGR register */
  // RCC1_PERIPH->CFGR = 0x00000000;

  // /* Reset HSEON, CSSON and PLLON bits */
  // RCC1_PERIPH->CR &= ( uint32_t )0xFEF6FFFF;

  // /* Reset PLLCFGR register */
  // RCC1_PERIPH->PLLCFGR = 0x24003010;

  // /* Reset HSEBYP bit */
  // RCC1_PERIPH->CR &= ( uint32_t )0xFFFBFFFF;

  // /* Disable all interrupts */
  // RCC1_PERIPH->CIR = 0x00000000;

  /*-------------------------------------------------
  Switch the system clock to be driven by the HSI
  -------------------------------------------------*/
  auto rcc = getCoreClockCtrl();
  rcc->enableClock( Chimera::Clock::Bus::HSI16 );
  rcc->setCoreClockSource( Chimera::Clock::Bus::HSI16 );

  /*------------------------------------------------
  Update the external clock variable so FreeRTOS
  can know the startup frequency.
  ------------------------------------------------*/
  CortexM4::Clock::updateCoreClockCache( getBusFrequency( Chimera::Clock::Bus::SYSCLK ) );

  /*-------------------------------------------------
  Enable some core peripheral clock modules needed
  for low level initialization code later.
  -------------------------------------------------*/
  Thor::LLD::PWR::clockEnable();
  Thor::LLD::SYS::clockEnable();
}
