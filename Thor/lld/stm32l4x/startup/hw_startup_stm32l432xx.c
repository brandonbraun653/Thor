/********************************************************************************
 *  File Name:
 *    hw_startup_stm32l432xx.c
 *
 *  Description:
 *    Startup vector table definition for the STM32L432
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#include <stddef.h>
#include <Thor/lld/stm32l4x/interrupt/hw_it_prj.hpp>

/*-------------------------------------------------
Beginning of the stack frame, as given by the linker script
-------------------------------------------------*/
extern void *_estack;

/*-------------------------------------------------
These are defined elsewhere
-------------------------------------------------*/
extern void Reset_Handler();

/*-------------------------------------------------
Provides default implementations of ISR handlers in case the
application code does not implement one. It's a nice catch.
-------------------------------------------------*/
#define DEBUG_DEFAULT_INTERRUPT_HANDLERS
#if defined( DEBUG_DEFAULT_INTERRUPT_HANDLERS )
void __attribute__( ( weak ) ) Reset_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) NMI_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) HardFault_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) MemManage_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) BusFault_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) UsageFault_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) SVCall_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DebugMonitor_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) PendSV_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) SysTick_Handler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) WWDG_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) PVD_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TAMP_STAMP_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) RTC_WKUP_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) FLASH_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) RCC_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI0_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI2_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI3_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI4_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream0_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream2_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream3_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream4_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream5_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream6_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) ADC_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN1_TX_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN1_RX0_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN1_RX1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN1_SCE_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI9_5_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM1_BRK_TIM9_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM1_UP_TIM10_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM1_TRG_COM_TIM11_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM1_CC_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM2_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM3_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C1_EV_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C1_ER_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C2_EV_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C2_ER_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) SPI1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) SPI2_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) USART1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) USART2_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) USART3_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI15_10_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) RTC_Alarm_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) SDMMC1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) SPI3_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) UART4_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM6_DAC_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM7_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream0_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream2_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream3_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream4_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DFSDM1_FLT0_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DFSDM1_FLT1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) COMP_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) LPTIM1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) LPTIM2_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) OTG_FS_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream5_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream6_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) LPUART1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) QUADSPI_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C3_EV_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C3_ER_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) SAI1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) SWPMI1_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) TSC_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) LCD_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) AES_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) RNG_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) FPU_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) CRS_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C4_EV_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C4_ER_IRQHandler()
{
  /* If you hit the breakpoint below, this particular interrupt was unhandled in your code.
     Define the handler in your code to manually override this one. */
  __asm__( "bkpt 255" );
}

#else
/*-------------------------------------------------
If the user doesn't specify a function for an ISR handler, default handle it.
-------------------------------------------------*/
void Reset_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void NMI_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void HardFault_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void MemManage_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void BusFault_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void UsageFault_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SVCall_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DebugMonitor_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void PendSV_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SysTick_Handler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void WWDG_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void PVD_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TAMP_STAMP_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void RTC_WKUP_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void FLASH_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void RCC_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI0_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI2_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI3_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI4_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream0_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream2_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream3_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream4_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream5_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream6_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void ADC_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN1_TX_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN1_RX0_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN1_RX1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN1_SCE_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI9_5_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM1_BRK_TIM9_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM1_UP_TIM10_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM1_TRG_COM_TIM11_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM1_CC_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM2_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM3_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C1_EV_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C1_ER_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C2_EV_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C2_ER_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SPI1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SPI2_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void USART1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void USART2_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void USART3_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI15_10_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void RTC_Alarm_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SDMMC1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SPI3_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void UART4_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM6_DAC_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM7_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream0_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream2_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream3_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream4_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DFSDM1_FLT0_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DFSDM1_FLT1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void COMP_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void LPTIM1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void LPTIM2_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void OTG_FS_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream5_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream6_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void LPUART1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void QUADSPI_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C3_EV_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C3_ER_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SAI1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SWPMI1_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TSC_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void LCD_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void AES_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void RNG_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void FPU_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CRS_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C4_EV_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C4_ER_IRQHandler __attribute__( ( weak, alias( "Default_Handler" ) ) );
#endif

/*-------------------------------------------------
These will always be defined regardless of the above macro
-------------------------------------------------*/
void __attribute__( ( noreturn ) ) Default_Handler()
{
  /* If you hit the breakpoint below, an ISR vector was left unhandled. Define DEBUG_DEFAULT_INTERRUPT_HANDLERS
     and recompile to figure out which one it is.*/
  __asm__( "bkpt 255" );
  for ( ;; )
  {
    ;
  }
}

/*-------------------------------------------------
The full vector table, placed at the linker script defined location ".isr_vector".
***MUST*** match the enumeration order of IRQn_Type. For full definition, see
RM0394 Section 12.3 "Interrupt and Exception Vectors"
-------------------------------------------------*/
/* clang-format off */
void *StartupVectorTable[] __attribute__( ( section( ".isr_vector" ), used ) ) = {
  /****** Cortex-M4 Processor Exception Handlers *******************************/
  &_estack,                 // xxx: Initial stack pointer value
  &Reset_Handler,           // xxx: Default function to invoke on any reset
  &NMI_Handler,             // -14: Non-Maskable Interrupt
  &HardFault_Handler,       // -13: Hard Fault
  &MemManage_Handler,       // -12: Memory Management Fault
  &BusFault_Handler,        // -11: Bus Fault
  &UsageFault_Handler,      // -10: Usage Fault
  NULL,                     //  -9: Reserved
  NULL,                     //  -8: Reserved
  NULL,                     //  -7: Reserved
  NULL,                     //  -6: Reserved
  &SVCall_Handler,          //  -5: Supervisor Call
  &DebugMonitor_Handler,    //  -4: Debug Monitor
  NULL,                     //  -3: Reserved
  &PendSV_Handler,          //  -2: Pending Supervisor Call
  &SysTick_Handler,         //  -1: System Tick
  /****** STM32 specific Interrupt Handlers ************************************/
  &WWDG_IRQHandler,         //   0
  &PVD_IRQHandler,
  &TAMP_STAMP_IRQHandler,
  &RTC_WKUP_IRQHandler,
  &FLASH_IRQHandler,
  &RCC_IRQHandler,
  &EXTI0_IRQHandler,
  &EXTI1_IRQHandler,
  &EXTI2_IRQHandler,
  &EXTI3_IRQHandler,
  &EXTI4_IRQHandler,        //  10
  &DMA1_Stream0_IRQHandler,
  &DMA1_Stream1_IRQHandler,
  &DMA1_Stream2_IRQHandler,
  &DMA1_Stream3_IRQHandler,
  &DMA1_Stream4_IRQHandler,
  &DMA1_Stream5_IRQHandler,
  &DMA1_Stream6_IRQHandler,
  &ADC_IRQHandler,
  &CAN1_TX_IRQHandler,
  &CAN1_RX0_IRQHandler,     //  20
  &CAN1_RX1_IRQHandler,
  &CAN1_SCE_IRQHandler,
  &EXTI9_5_IRQHandler,
  &TIM1_BRK_TIM9_IRQHandler,
  &TIM1_UP_TIM10_IRQHandler,
  &TIM1_TRG_COM_TIM11_IRQHandler,
  &TIM1_CC_IRQHandler,
  &TIM2_IRQHandler,
  &TIM3_IRQHandler,
  NULL,                     //  30
  &I2C1_EV_IRQHandler,
  &I2C1_ER_IRQHandler,
  &I2C2_EV_IRQHandler,
  &I2C2_ER_IRQHandler,
  &SPI1_IRQHandler,
  &SPI2_IRQHandler,
  &USART1_IRQHandler,
  &USART2_IRQHandler,
  &USART3_IRQHandler,
  &EXTI15_10_IRQHandler,    //  40
  &RTC_Alarm_IRQHandler,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  &SDMMC1_IRQHandler,
  NULL,                     //  50
  &SPI3_IRQHandler,
  &UART4_IRQHandler,
  NULL,
  &TIM6_DAC_IRQHandler,
  &TIM7_IRQHandler,
  &DMA2_Stream0_IRQHandler,
  &DMA2_Stream1_IRQHandler,
  &DMA2_Stream2_IRQHandler,
  &DMA2_Stream3_IRQHandler,
  &DMA2_Stream4_IRQHandler, //  60
  &DFSDM1_FLT0_IRQHandler,
  &DFSDM1_FLT1_IRQHandler,
  NULL,
  &COMP_IRQHandler,
  &LPTIM1_IRQHandler,
  &LPTIM2_IRQHandler,
  &OTG_FS_IRQHandler,
  &DMA2_Stream5_IRQHandler,
  &DMA2_Stream6_IRQHandler,
  &LPUART1_IRQHandler,      //  70
  &QUADSPI_IRQHandler,
  &I2C3_EV_IRQHandler,
  &I2C3_ER_IRQHandler,
  &SAI1_IRQHandler,
  NULL,
  &SWPMI1_IRQHandler,
  &TSC_IRQHandler,
  &LCD_IRQHandler,
  &AES_IRQHandler,
  &RNG_IRQHandler,          //  80
  &FPU_IRQHandler,
  &CRS_IRQHandler,
  &I2C4_EV_IRQHandler,
  &I2C4_ER_IRQHandler
};
/* clang-format on */