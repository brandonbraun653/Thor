/********************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32l432kc.hpp
 *
 *  Description:
 *    GPIO register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_SIM_VARIANT_HPP
#define THOR_LLD_GPIO_SIM_VARIANT_HPP

/* C++ Includes */
#include <cstdint>
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <Thor/lld/interface/system/sim/system_sim_memory_map.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_GPIO )

/*-------------------------------------------------
Peripheral Availability
-------------------------------------------------*/
#define STM32_GPIOA_PERIPH_AVAILABLE
#define STM32_GPIOB_PERIPH_AVAILABLE
#define STM32_GPIOC_PERIPH_AVAILABLE

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_GPIO_PERIPHS = 3;   /**< Supported GPIO peripherals */
  static constexpr size_t NUM_GPIO_PINS    = 25;  /**< Max available pins to be configured as GPIO */

  static constexpr RIndex_t GPIOA_RESOURCE_INDEX = 0;
  static constexpr RIndex_t GPIOB_RESOURCE_INDEX = 1;
  static constexpr RIndex_t GPIOC_RESOURCE_INDEX = 2;
  static constexpr RIndex_t GPIOD_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOE_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOF_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOG_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOH_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOI_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOJ_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOK_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOL_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;

  static constexpr uint8_t GPIOA_NUM_PINS = 16;
  static constexpr uint8_t GPIOB_NUM_PINS = 7;
  static constexpr uint8_t GPIOC_NUM_PINS = 2;
  static constexpr uint8_t GPIOD_NUM_PINS = 0;
  static constexpr uint8_t GPIOE_NUM_PINS = 0;
  static constexpr uint8_t GPIOF_NUM_PINS = 0;
  static constexpr uint8_t GPIOG_NUM_PINS = 0;
  static constexpr uint8_t GPIOH_NUM_PINS = 0;
  static constexpr uint8_t GPIOI_NUM_PINS = 0;
  static constexpr uint8_t GPIOJ_NUM_PINS = 0;
  static constexpr uint8_t GPIOK_NUM_PINS = 0;
  static constexpr uint8_t GPIOL_NUM_PINS = 0;

  static constexpr uint8_t PRJ_MAX_PORTS = NUM_GPIO_PERIPHS;
  static constexpr uint8_t PRJ_MAX_PINS  = NUM_GPIO_PINS;

    /**
   *  Initializes the LLD register resources and memory
   *
   *  @return void
   */
  void initializeRegisters();

  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t GPIOA_BASE_ADDR = Thor::System::MemoryMap::GPIOA_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOB_BASE_ADDR = Thor::System::MemoryMap::GPIOB_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOC_BASE_ADDR = Thor::System::MemoryMap::GPIOC_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOD_BASE_ADDR = Thor::System::MemoryMap::GPIOD_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOE_BASE_ADDR = Thor::System::MemoryMap::GPIOE_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOH_BASE_ADDR = Thor::System::MemoryMap::GPIOH_PERIPH_START_ADDRESS;

  /*------------------------------------------------
  Alternate Functions
  ------------------------------------------------*/
  static constexpr Reg8_t AF0_RTC_50Hz   = 0x00; /* RTC_50Hz Alternate Function mapping */
  static constexpr Reg8_t AF0_MCO        = 0x00; /* MCO (MCO1 and MCO2) Alternate Function mapping */
  static constexpr Reg8_t AF0_SWJ        = 0x00; /* SWJ (SWD and JTAG) Alternate Function mapping */
  static constexpr Reg8_t AF0_TRACE      = 0x00; /* TRACE Alternate Function mapping */
  static constexpr Reg8_t AF1_TIM1       = 0x01; /* TIM1 Alternate Function mapping */
  static constexpr Reg8_t AF1_TIM2       = 0x01; /* TIM2 Alternate Function mapping */
  static constexpr Reg8_t AF1_LPTIM1     = 0x01; /* LPTIM1 Alternate Function mapping */
  static constexpr Reg8_t AF1_IR         = 0x01; /* IR Alternate Function mapping */
  static constexpr Reg8_t AF2_TIM1       = 0x02; /* TIM1 Alternate Function mapping */
  static constexpr Reg8_t AF2_TIM2       = 0x02; /* TIM2 Alternate Function mapping */
  static constexpr Reg8_t AF3_USART2     = 0x03; /* USART1 Alternate Function mapping */
  static constexpr Reg8_t AF3_TIM1_COMP2 = 0x03; /* TIM1/COMP2 Break in Alternate Function mapping */
  static constexpr Reg8_t AF3_TIM1_COMP1 = 0x03; /* TIM1/COMP1 Break in Alternate Function mapping */
  static constexpr Reg8_t AF4_I2C1       = 0x04; /* I2C1 Alternate Function mapping */
  static constexpr Reg8_t AF4_I2C2       = 0x04; /* I2C2 Alternate Function mapping */
  static constexpr Reg8_t AF4_I2C3       = 0x04; /* I2C3 Alternate Function mapping */
  static constexpr Reg8_t AF5_SPI1       = 0x05; /* SPI1 Alternate Function mapping */
  static constexpr Reg8_t AF5_SPI2       = 0x05; /* SPI2 Alternate Function mapping */
  static constexpr Reg8_t AF6_SPI3       = 0x06; /* SPI3 Alternate Function mapping */
  static constexpr Reg8_t AF6_COMP1      = 0x06; /* COMP1 Alternate Function mapping */
  static constexpr Reg8_t AF7_USART1     = 0x07; /* USART1 Alternate Function mapping */
  static constexpr Reg8_t AF7_USART2     = 0x07; /* USART2 Alternate Function mapping */
  static constexpr Reg8_t AF7_USART3     = 0x07; /* USART3 Alternate Function mapping */
  static constexpr Reg8_t AF8_LPUART1    = 0x08; /* LPUART1 Alternate Function mapping */
  static constexpr Reg8_t AF9_CAN1       = 0x09; /* CAN1 Alternate Function mapping */
  static constexpr Reg8_t AF9_TSC        = 0x09; /* TSC Alternate Function mapping */
  static constexpr Reg8_t AF10_USB_FS    = 0x0A; /* USB_FS Alternate Function mapping */
  static constexpr Reg8_t AF10_QUADSPI   = 0x0A; /* QUADSPI Alternate Function mapping */
  static constexpr Reg8_t AF12_SWPMI1    = 0x0C; /* SWPMI1 Alternate Function mapping */
  static constexpr Reg8_t AF12_COMP1     = 0x0C; /* COMP1 Alternate Function mapping */
  static constexpr Reg8_t AF12_COMP2     = 0x0C; /* COMP2 Alternate Function mapping */
  static constexpr Reg8_t AF12_SDMMC1    = 0x0C; /* SDMMC1 Alternate Function mapping */
  static constexpr Reg8_t AF13_SAI1      = 0x0D; /* SAI1 Alternate Function mapping */
  static constexpr Reg8_t AF14_TIM2      = 0x0E; /* TIM2 Alternate Function mapping */
  static constexpr Reg8_t AF14_TIM15     = 0x0E; /* TIM15 Alternate Function mapping */
  static constexpr Reg8_t AF14_TIM16     = 0x0E; /* TIM16 Alternate Function mapping */
  static constexpr Reg8_t AF14_LPTIM2    = 0x0E; /* LPTIM2 Alternate Function mapping */
  static constexpr Reg8_t AF15_EVENTOUT  = 0x0F; /* EVENTOUT Alternate Function mapping */

  /*------------------------------------------------
  Register Pin Mapping
  ------------------------------------------------*/
  static constexpr uint32_t PIN_0        = 0x0001; /**< Pin 0 selected    */
  static constexpr uint32_t PIN_1        = 0x0002; /**< Pin 1 selected    */
  static constexpr uint32_t PIN_2        = 0x0004; /**< Pin 2 selected    */
  static constexpr uint32_t PIN_3        = 0x0008; /**< Pin 3 selected    */
  static constexpr uint32_t PIN_4        = 0x0010; /**< Pin 4 selected    */
  static constexpr uint32_t PIN_5        = 0x0020; /**< Pin 5 selected    */
  static constexpr uint32_t PIN_6        = 0x0040; /**< Pin 6 selected    */
  static constexpr uint32_t PIN_7        = 0x0080; /**< Pin 7 selected    */
  static constexpr uint32_t PIN_8        = 0x0100; /**< Pin 8 selected    */
  static constexpr uint32_t PIN_9        = 0x0200; /**< Pin 9 selected    */
  static constexpr uint32_t PIN_10       = 0x0400; /**< Pin 10 selected   */
  static constexpr uint32_t PIN_11       = 0x0800; /**< Pin 11 selected   */
  static constexpr uint32_t PIN_12       = 0x1000; /**< Pin 12 selected   */
  static constexpr uint32_t PIN_13       = 0x2000; /**< Pin 13 selected   */
  static constexpr uint32_t PIN_14       = 0x4000; /**< Pin 14 selected   */
  static constexpr uint32_t PIN_15       = 0x8000; /**< Pin 15 selected   */
  static constexpr uint32_t PIN_All      = 0xFFFF; /**< All pins selected */
  static constexpr uint32_t MAX_NUM_PINS = 16;


}    // namespace Thor::LLD::GPIO

#endif 
#endif /* !THOR_HW_GPIO_REGISTER_STM32L432KC_HPP */