/********************************************************************************
 *  File Name:
 *    hw_rcc_mapping.hpp
 *
 *  Description:
 *    Defines structures used to map various resources needed in the RCC driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_MAPPING_HPP
#define THOR_LLD_RCC_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/clock>
#include <Chimera/common>
#include <Chimera/container>

/* Driver Includes */
#include <Thor/lld/stm32l4x/adc/hw_adc_prj.hpp>
#include <Thor/lld/stm32l4x/can/hw_can_prj.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_prj.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32l4x/system/hw_sys_prj.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/system/sys_memory_map_prj.hpp>

namespace Thor::LLD::RCC
{
#if defined( STM32_RCC1_PERIPH_AVAILABLE )
  extern RegisterMap *RCC1_PERIPH;
#endif

  /*------------------------------------------------
  Peripheral Memory Mapping
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Thor::LLD::RIndexMap InstanceToResourceIndex;

  /**
   *  Initializes memory associated with mapping
   *
   *  @return void
   */
  extern void initializeMapping();

  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  extern bool isRCC( const std::uintptr_t address );


  namespace LookupTables
  {
#if defined( THOR_LLD_ADC )
    extern void ADCInit();
    extern PCC ADCLookup;
    extern RegisterConfig ADC_ClockConfig[ Thor::LLD::ADC::NUM_ADC_PERIPHS ];
    extern RegisterConfig ADC_ResetConfig[ Thor::LLD::ADC::NUM_ADC_PERIPHS ];
    extern Chimera::Clock::Bus ADC_SourceClock[ Thor::LLD::ADC::NUM_ADC_PERIPHS ];
#endif /* THOR_LLD_ADC */

#if defined( THOR_LLD_CAN )
    extern void CANInit();
    extern PCC CANLookup;
    extern RegisterConfig CAN_ClockConfig[ Thor::LLD::CAN::NUM_CAN_PERIPHS ];
    extern RegisterConfig CAN_ResetConfig[ Thor::LLD::CAN::NUM_CAN_PERIPHS ];
    extern Chimera::Clock::Bus CAN_SourceClock[ Thor::LLD::CAN::NUM_CAN_PERIPHS ];
#endif /* THOR_LLD_CAN */

#if defined( THOR_LLD_FLASH )
    extern void FLASHInit();
    extern PCC FLASHLookup;
    extern RegisterConfig FLASH_ClockConfig[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
    extern RegisterConfig FLASH_ResetConfig[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
    extern Chimera::Clock::Bus FLASH_SourceClock[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
#endif /* THOR_LLD_FLASH */

#if defined( THOR_LLD_GPIO )
    extern void GPIOInit();
    extern PCC GPIOLookup;
    extern RegisterConfig GPIO_ClockConfig[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
    extern RegisterConfig GPIO_ResetConfig[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
    extern Chimera::Clock::Bus GPIO_SourceClock[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
#endif /* THOR_LLD_GPIO */

#if defined( THOR_LLD_PWR )
    extern void PWRInit();
    extern PCC PWRLookup;
    extern RegisterConfig PWR_ClockConfig[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];
    extern RegisterConfig PWR_ResetConfig[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];
    extern Chimera::Clock::Bus PWR_SourceClock[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];
#endif /* THOR_LLD_PWR */

#if defined( THOR_LLD_SPI )
    extern void SPIInit();
    extern PCC SPILookup;
    extern RegisterConfig SPI_ClockConfig[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
    extern RegisterConfig SPI_ResetConfig[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
    extern Chimera::Clock::Bus SPI_SourceClock[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
#endif /* THOR_LLD_SPI */

#if defined( THOR_LLD_SYSCFG )
    extern void SYSCFGInit();
    extern PCC SYSCFGLookup;
    extern RegisterConfig SYSCFG_ClockConfig[ Thor::LLD::SYS::NUM_SYSCFG_PERIPHS ];
    extern RegisterConfig SYSCFG_ResetConfig[ Thor::LLD::SYS::NUM_SYSCFG_PERIPHS ];
    extern Chimera::Clock::Bus SYSCFG_SourceClock[ Thor::LLD::SYS::NUM_SYSCFG_PERIPHS ];
#endif /* THOR_LLD_SYSCFG */

#if defined( THOR_LLD_TIMER )
    extern void TIMERInit();
    extern PCC TIMERLookup;
    extern RegisterConfig TIMER_ClockConfig[ Thor::LLD::TIMER::NUM_TIMER_PERIPHS ];
    extern RegisterConfig TIMER_ResetConfig[ Thor::LLD::TIMER::NUM_TIMER_PERIPHS ];
    extern Chimera::Clock::Bus TIMER_SourceClock[ Thor::LLD::TIMER::NUM_TIMER_PERIPHS ];
#endif /* THOR_LLD_TIMER */

#if defined( THOR_LLD_USART )
    extern void USARTInit();
    extern PCC USARTLookup;
    extern RegisterConfig USART_ClockConfig[ Thor::LLD::USART::NUM_USART_PERIPHS ];
    extern RegisterConfig USART_ResetConfig[ Thor::LLD::USART::NUM_USART_PERIPHS ];
    extern Chimera::Clock::Bus USART_SourceClock[ Thor::LLD::USART::NUM_USART_PERIPHS ];
#endif /* THOR_LLD_USART */
  }    // namespace LookupTables
}    // namespace Thor::LLD::RCC

#endif /* !THOR_LLD_RCC_MAPPING_HPP */
