/********************************************************************************
 *   File Name:
 *     sys_common_init.cpp
 *
 *   Description:
 *     Provides STM32F4xx generic initialization functionality
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include "thor_config_prj.hpp"
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>

/* ARM Includes: Must come last so the Thor Includes can configure various macros */
#include <Thor/drivers/common/cmsis/core/include/core_cm4.h>

using namespace Thor::Driver::RCC;

void SystemInit()
{
/*
  1. reset the RCC periperal
  2. Configure the base user clock
  3. Uh...custom init step??
*/

/* FPU settings ------------------------------------------------------------*/
#if ( __FPU_PRESENT == 1 ) && ( __FPU_USED == 1 )
  SCB->CPACR |= ( ( 3UL << 10 * 2 ) | ( 3UL << 11 * 2 ) ); /* set CP10 and CP11 Full Access */
#endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC_PERIPH->CR |= ( uint32_t )0x00000001;

  /* Reset CFGR register */
  RCC_PERIPH->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC_PERIPH->CR &= ( uint32_t )0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC_PERIPH->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC_PERIPH->CR &= ( uint32_t )0xFFFBFFFF;

  /* Disable all interrupts */
  RCC_PERIPH->CIR = 0x00000000;

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = Thor::System::MemoryMap::FLASH_BASE_ADDR |
              Thor::System::MemoryMap::VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}