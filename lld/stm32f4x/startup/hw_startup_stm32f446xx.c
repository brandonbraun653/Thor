/******************************************************************************
 *  File Name:
 *    hw_startup_stm32f446xx.c
 *
 *  Description:
 *    This file contains the entry point (Reset_Handler) of your firmware project.
 *    The reset handled initializes the RAM and calls system library initializers
 *    as well as the platform-specific initializer and the main() function.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#if defined( EMBEDDED )

#include <stddef.h>
extern void *_estack;

extern void Reset_Handler();
extern void Default_Handler();

#define DEBUG_DEFAULT_INTERRUPT_HANDLERS
#ifdef DEBUG_DEFAULT_INTERRUPT_HANDLERS
void __attribute__( ( weak ) ) NMI_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void NMI_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) HardFault_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void HardFault_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) MemManage_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void MemManage_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) BusFault_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void BusFault_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) UsageFault_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void UsageFault_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SVC_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SVC_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DebugMon_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DebugMon_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) PendSV_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void PendSV_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SysTick_Handler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SysTick_Handler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) WWDG_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void WWDG_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) PVD_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void PVD_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TAMP_STAMP_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TAMP_STAMP_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) RTC_WKUP_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void RTC_WKUP_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) FLASH_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void FLASH_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) RCC_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void RCC_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI0_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void EXTI0_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI1_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void EXTI1_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI2_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void EXTI2_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI3_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void EXTI3_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI4_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void EXTI4_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream0_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA1_Stream0_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream1_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA1_Stream1_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream2_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA1_Stream2_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream3_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA1_Stream3_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream4_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA1_Stream4_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream5_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA1_Stream5_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream6_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA1_Stream6_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) ADC_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void ADC_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN1_TX_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CAN1_TX_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN1_RX0_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CAN1_RX0_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN1_RX1_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CAN1_RX1_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN1_SCE_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CAN1_SCE_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI9_5_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void EXTI9_5_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM1_BRK_TIM9_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM1_BRK_TIM9_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM1_UP_TIM10_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM1_UP_TIM10_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM1_TRG_COM_TIM11_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM1_TRG_COM_TIM11_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM1_CC_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM1_CC_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM2_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM2_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM3_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM3_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM4_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM4_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C1_EV_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void I2C1_EV_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C1_ER_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void I2C1_ER_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C2_EV_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void I2C2_EV_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C2_ER_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void I2C2_ER_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SPI1_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SPI1_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SPI2_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SPI2_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) USART1_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void USART1_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) USART2_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void USART2_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) USART3_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void USART3_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) EXTI15_10_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void EXTI15_10_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) RTC_Alarm_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void RTC_Alarm_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) OTG_FS_WKUP_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void OTG_FS_WKUP_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM8_BRK_TIM12_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM8_BRK_TIM12_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM8_UP_TIM13_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM8_UP_TIM13_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM8_TRG_COM_TIM14_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM8_TRG_COM_TIM14_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM8_CC_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM8_CC_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA1_Stream7_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA1_Stream7_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) FMC_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void FMC_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SDIO_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SDIO_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM5_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM5_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SPI3_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SPI3_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) UART4_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void UART4_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) UART5_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void UART5_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM6_DAC_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM6_DAC_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) TIM7_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void TIM7_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream0_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA2_Stream0_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream1_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA2_Stream1_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream2_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA2_Stream2_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream3_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA2_Stream3_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream4_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA2_Stream4_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN2_TX_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CAN2_TX_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN2_RX0_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CAN2_RX0_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN2_RX1_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CAN2_RX1_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CAN2_SCE_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CAN2_SCE_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) OTG_FS_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void OTG_FS_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream5_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA2_Stream5_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream6_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA2_Stream6_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DMA2_Stream7_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DMA2_Stream7_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) USART6_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void USART6_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C3_EV_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void I2C3_EV_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) I2C3_ER_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void I2C3_ER_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) OTG_HS_EP1_OUT_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void OTG_HS_EP1_OUT_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) OTG_HS_EP1_IN_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void OTG_HS_EP1_IN_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) OTG_HS_WKUP_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void OTG_HS_WKUP_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) OTG_HS_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void OTG_HS_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) DCMI_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void DCMI_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) FPU_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void FPU_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SPI4_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SPI4_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SAI1_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SAI1_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SAI2_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SAI2_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) QUADSPI_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void QUADSPI_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) CEC_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void CEC_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) SPDIF_RX_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void SPDIF_RX_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) FMPI2C1_Event_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void FMPI2C1_Event_IRQHandler();
  asm( "bkpt 255" );
}

void __attribute__( ( weak ) ) FMPI2C1_Error_IRQHandler()
{
  // If you hit the breakpoint below, one of the interrupts was unhandled in your code.
  // Define the following function in your code to handle it:
  //	extern "C" void FMPI2C1_Error_IRQHandler();
  asm( "bkpt 255" );
}

#else
void NMI_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void HardFault_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void MemManage_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void BusFault_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void UsageFault_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SVC_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DebugMon_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void PendSV_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SysTick_Handler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void WWDG_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void PVD_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TAMP_STAMP_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void RTC_WKUP_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void FLASH_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void RCC_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI0_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI1_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI2_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI3_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI4_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream0_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream1_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream2_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream3_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream4_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream5_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream6_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void ADC_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN1_TX_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN1_RX0_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN1_RX1_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN1_SCE_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI9_5_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM1_BRK_TIM9_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM1_UP_TIM10_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM1_TRG_COM_TIM11_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM1_CC_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM2_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM3_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM4_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C1_EV_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C1_ER_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C2_EV_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C2_ER_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SPI1_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SPI2_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void USART1_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void USART2_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void USART3_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void EXTI15_10_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void RTC_Alarm_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void OTG_FS_WKUP_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM8_BRK_TIM12_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM8_UP_TIM13_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM8_TRG_COM_TIM14_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM8_CC_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA1_Stream7_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void FMC_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SDIO_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM5_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SPI3_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void UART4_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void UART5_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM6_DAC_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void TIM7_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream0_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream1_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream2_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream3_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream4_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN2_TX_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN2_RX0_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN2_RX1_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CAN2_SCE_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void OTG_FS_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream5_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream6_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DMA2_Stream7_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void USART6_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C3_EV_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void I2C3_ER_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void OTG_HS_EP1_OUT_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void OTG_HS_EP1_IN_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void OTG_HS_WKUP_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void OTG_HS_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void DCMI_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void FPU_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SPI4_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SAI1_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SAI2_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void QUADSPI_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void CEC_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void SPDIF_RX_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void FMPI2C1_Event_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
void FMPI2C1_Error_IRQHandler() __attribute__( ( weak, alias( "Default_Handler" ) ) );
#endif


void __attribute__( ( noreturn ) ) Default_Handler()
{
  // If you get stuck here, your code is missing a handler for some interrupt.
  // Define a 'DEBUG_DEFAULT_INTERRUPT_HANDLERS' macro via VisualGDB Project Properties and rebuild your project.
  // This will pinpoint a specific missing vector.
  asm( "bkpt 255" );
  for ( ;; )
    ;
}

/*-------------------------------------------------
The full vector table, placed at the linker script defined location ".isr_vector".
***MUST*** match the enumeration order of IRQn_Type. For full definition, see
RM0394 Section 12.3 "Interrupt and Exception Vectors"
-------------------------------------------------*/
/* clang-format off */
void *StartupVectorTable[] __attribute__( ( section( ".isr_vector" ), used ) ) = {
  /****** Cortex-M4 Processor Exception Handlers *******************************/
  &_estack,               // xxx: Initial stack pointer value
  &Reset_Handler,         // xxx: Default function to invoke on any reset
  &NMI_Handler,           // -14: Non-Maskable Interrupt
  &HardFault_Handler,     // -13: Hard Fault
  &MemManage_Handler,     // -12: Memory Management Fault
  &BusFault_Handler,      // -11: Bus Fault
  &UsageFault_Handler,    // -10: Usage Fault
  NULL,                   //  -9: Reserved
  NULL,                   //  -8: Reserved
  NULL,                   //  -7: Reserved
  NULL,                   //  -6: Reserved
  &SVC_Handler,           //  -5: Supervisor Call
  &DebugMon_Handler,      //  -4: Debug Monitor
  NULL,                   //  -3: Reserved
  &PendSV_Handler,        //  -2: Pending Supervisor Call
  &SysTick_Handler,       //  -1: System Tick
  /****** STM32 specific Interrupt Handlers ************************************/
  &WWDG_IRQHandler,
  &PVD_IRQHandler,
  &TAMP_STAMP_IRQHandler,
  &RTC_WKUP_IRQHandler,
  &FLASH_IRQHandler,
  &RCC_IRQHandler,
  &EXTI0_IRQHandler,
  &EXTI1_IRQHandler,
  &EXTI2_IRQHandler,
  &EXTI3_IRQHandler,
  &EXTI4_IRQHandler,
  &DMA1_Stream0_IRQHandler,
  &DMA1_Stream1_IRQHandler,
  &DMA1_Stream2_IRQHandler,
  &DMA1_Stream3_IRQHandler,
  &DMA1_Stream4_IRQHandler,
  &DMA1_Stream5_IRQHandler,
  &DMA1_Stream6_IRQHandler,
  &ADC_IRQHandler,
  &CAN1_TX_IRQHandler,
  &CAN1_RX0_IRQHandler,
  &CAN1_RX1_IRQHandler,
  &CAN1_SCE_IRQHandler,
  &EXTI9_5_IRQHandler,
  &TIM1_BRK_TIM9_IRQHandler,
  &TIM1_UP_TIM10_IRQHandler,
  &TIM1_TRG_COM_TIM11_IRQHandler,
  &TIM1_CC_IRQHandler,
  &TIM2_IRQHandler,
  &TIM3_IRQHandler,
  &TIM4_IRQHandler,
  &I2C1_EV_IRQHandler,
  &I2C1_ER_IRQHandler,
  &I2C2_EV_IRQHandler,
  &I2C2_ER_IRQHandler,
  &SPI1_IRQHandler,
  &SPI2_IRQHandler,
  &USART1_IRQHandler,
  &USART2_IRQHandler,
  &USART3_IRQHandler,
  &EXTI15_10_IRQHandler,
  &RTC_Alarm_IRQHandler,
  &OTG_FS_WKUP_IRQHandler,
  &TIM8_BRK_TIM12_IRQHandler,
  &TIM8_UP_TIM13_IRQHandler,
  &TIM8_TRG_COM_TIM14_IRQHandler,
  &TIM8_CC_IRQHandler,
  &DMA1_Stream7_IRQHandler,
  &FMC_IRQHandler,
  &SDIO_IRQHandler,
  &TIM5_IRQHandler,
  &SPI3_IRQHandler,
  &UART4_IRQHandler,
  &UART5_IRQHandler,
  &TIM6_DAC_IRQHandler,
  &TIM7_IRQHandler,
  &DMA2_Stream0_IRQHandler,
  &DMA2_Stream1_IRQHandler,
  &DMA2_Stream2_IRQHandler,
  &DMA2_Stream3_IRQHandler,
  &DMA2_Stream4_IRQHandler,
  NULL,
  NULL,
  &CAN2_TX_IRQHandler,
  &CAN2_RX0_IRQHandler,
  &CAN2_RX1_IRQHandler,
  &CAN2_SCE_IRQHandler,
  &OTG_FS_IRQHandler,
  &DMA2_Stream5_IRQHandler,
  &DMA2_Stream6_IRQHandler,
  &DMA2_Stream7_IRQHandler,
  &USART6_IRQHandler,
  &I2C3_EV_IRQHandler,
  &I2C3_ER_IRQHandler,
  &OTG_HS_EP1_OUT_IRQHandler,
  &OTG_HS_EP1_IN_IRQHandler,
  &OTG_HS_WKUP_IRQHandler,
  &OTG_HS_IRQHandler,
  &DCMI_IRQHandler,
  NULL,
  NULL,
  &FPU_IRQHandler,
  NULL,
  NULL,
  &SPI4_IRQHandler,
  NULL,
  NULL,
  &SAI1_IRQHandler,
  NULL,
  NULL,
  NULL,
  &SAI2_IRQHandler,
  &QUADSPI_IRQHandler,
  &CEC_IRQHandler,
  &SPDIF_RX_IRQHandler,
  &FMPI2C1_Event_IRQHandler,
  &FMPI2C1_Error_IRQHandler,
};
/* clang-format on */

#endif /* _EMBEDDED */