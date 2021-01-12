/********************************************************************************
 *  File Name:
 *    hw_interrupt_register_stm32l432kc.hpp
 *
 *  Description:
 *    INTERRUPT register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_INTERRUPT_REGISTER_STM32L432KC_HPP
#define THOR_HW_INTERRUPT_REGISTER_STM32L432KC_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>

/*-------------------------------------------------
ISR vector numbers as described in PM0214 Section 2.3.4 and RM0394 Section 12.3.
-------------------------------------------------*/
enum IRQn_Type : int
{
  /****** Cortex-M4 Processor Exceptions Numbers ******/
  NonMaskableInt_IRQn   = -14,
  MemoryManagement_IRQn = -12,
  BusFault_IRQn         = -11,
  UsageFault_IRQn       = -10,
  SVCall_IRQn           = -5,
  DebugMonitor_IRQn     = -4,
  PendSV_IRQn           = -2,
  SysTick_IRQn          = -1,
  /****** STM32 specific Interrupt Numbers ******/
  WWDG_IRQn               = 0,
  PVD_IRQn                = 1,
  TAMP_STAMP_IRQn         = 2,
  RTC_WKUP_IRQn           = 3,
  FLASH_IRQn              = 4,
  RCC_IRQn                = 5,
  EXTI0_IRQn              = 6,
  EXTI1_IRQn              = 7,
  EXTI2_IRQn              = 8,
  EXTI3_IRQn              = 9,
  EXTI4_IRQn              = 10,
  DMA1_Stream0_IRQn       = 11,
  DMA1_Stream1_IRQn       = 12,
  DMA1_Stream2_IRQn       = 13,
  DMA1_Stream3_IRQn       = 14,
  DMA1_Stream4_IRQn       = 15,
  DMA1_Stream5_IRQn       = 16,
  DMA1_Stream6_IRQn       = 17,
  ADC_IRQn                = 18,
  CAN1_TX_IRQn            = 19,
  CAN1_RX0_IRQn           = 20,
  CAN1_RX1_IRQn           = 21,
  CAN1_SCE_IRQn           = 22,
  EXTI9_5_IRQn            = 23,
  TIM1_BRK_TIM9_IRQn      = 24,
  TIM1_UP_TIM10_IRQn      = 25,
  TIM1_TRG_COM_TIM11_IRQn = 26,
  TIM1_CC_IRQn            = 27,
  TIM2_IRQn               = 28,
  TIM3_IRQn               = 29,
  I2C1_EV_IRQn            = 31,
  I2C1_ER_IRQn            = 32,
  I2C2_EV_IRQn            = 33,
  I2C2_ER_IRQn            = 34,
  SPI1_IRQn               = 35,
  SPI2_IRQn               = 36,
  USART1_IRQn             = 37,
  USART2_IRQn             = 38,
  USART3_IRQn             = 39,
  EXTI15_10_IRQn          = 40,
  RTC_Alarm_IRQn          = 41,
  SDMMC1_IRQn             = 49,
  SPI3_IRQn               = 51,
  UART4_IRQn              = 52,
  TIM6_DAC_IRQn           = 54,
  TIM7_IRQn               = 55,
  DMA2_Stream0_IRQn       = 56,
  DMA2_Stream1_IRQn       = 57,
  DMA2_Stream2_IRQn       = 58,
  DMA2_Stream3_IRQn       = 59,
  DMA2_Stream4_IRQn       = 60,
  DFSDM1_FLT0_IRQn        = 61,
  DFSDM1_FLT1_IRQn        = 62,
  COMP_IRQn               = 64,
  LPTIM1_IRQn             = 65,
  LPTIM2_IRQn             = 66,
  OTG_FS_IRQn             = 67,
  DMA2_Stream5_IRQn       = 68,
  DMA2_Stream6_IRQn       = 69,
  LPUART1_IRQn            = 70,
  QUADSPI_IRQn            = 71,
  I2C3_EV_IRQn            = 72,
  I2C3_ER_IRQn            = 73,
  SAI1_IRQn               = 74,
  SWPMI1_IRQn             = 76,
  TSC_IRQn                = 77,
  LCD_IRQn                = 78,
  AES_IRQn                = 79,
  RNG_IRQn                = 80,
  FPU_IRQn                = 81,
  CRS_IRQn                = 82,
  I2C4_EV_IRQn            = 83,
  I2C4_ER_IRn             = 84
};


namespace Thor::LLD::IT
{
}    // namespace Thor::LLD::IT

#endif /* !THOR_HW_INTERRUPT_REGISTER_STM32L432KC_HPP */