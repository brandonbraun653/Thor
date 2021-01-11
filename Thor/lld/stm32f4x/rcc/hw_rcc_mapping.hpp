/********************************************************************************
 *  File Name:
 *    hw_rcc_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_RCC_MAPPING_HPP
#define THOR_HW_RCC_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/container>
#include <Chimera/gpio>

/* Driver Includes */
#include <Thor/lld/stm32f4x/dma/hw_dma_prj.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_prj.hpp>
#include <Thor/lld/stm32f4x/power/hw_power_prj.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32f4x/system/sys_memory_map_prj.hpp>
#include <Thor/lld/stm32f4x/uart/hw_uart_prj.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_prj.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_prj.hpp>

namespace Thor::LLD::RCC
{
#if defined( STM32_RCC1_PERIPH_AVAILABLE )
  extern RegisterMap *RCC1_PERIPH;
#endif

  /*------------------------------------------------
  Peripheral Memory Mapping
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;

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

#if defined( THOR_LLD_CRS )
    extern void CRSInit();

    extern PCC CRSLookup;
    extern RegisterConfig CRS_ClockConfig[ Thor::LLD::CRS::NUM_CRS_PERIPHS ];
    extern RegisterConfig CRS_ResetConfig[ Thor::LLD::CRS::NUM_CRS_PERIPHS ];
    extern Chimera::Clock::Bus CRS_SourceClock[ Thor::LLD::CRS::NUM_CRS_PERIPHS ];
#endif /* THOR_LLD_CRS */

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

#if defined( THOR_LLD_IWDG )
    extern void IWDGInit();
    extern PCC IWDGLookup;
    extern RegisterConfig IWDG_ClockConfig[ Thor::LLD::IWDG::NUM_IWDG_PERIPHS ];
    extern RegisterConfig IWDG_ResetConfig[ Thor::LLD::IWDG::NUM_IWDG_PERIPHS ];
    extern Chimera::Clock::Bus IWDG_SourceClock[ Thor::LLD::IWDG::NUM_IWDG_PERIPHS ];
#endif /* THOR_LLD_IWDG */

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

#if defined( THOR_LLD_USB )
    extern void USBInit();
    extern PCC USBLookup;
    extern RegisterConfig USB_ClockConfig[ Thor::LLD::USB::NUM_USB_PERIPHS ];
    extern RegisterConfig USB_ResetConfig[ Thor::LLD::USB::NUM_USB_PERIPHS ];
    extern Chimera::Clock::Bus USB_SourceClock[ Thor::LLD::USB::NUM_USB_PERIPHS ];
#endif /* THOR_LLD_USB */

#if defined( THOR_LLD_WWDG )
    extern void WWDGInit();
    extern PCC WWDGLookup;
    extern RegisterConfig WWDG_ClockConfig[ Thor::LLD::WWDG::NUM_WWDG_PERIPHS ];
    extern RegisterConfig WWDG_ResetConfig[ Thor::LLD::WWDG::NUM_WWDG_PERIPHS ];
    extern Chimera::Clock::Bus WWDG_SourceClock[ Thor::LLD::WWDG::NUM_WWDG_PERIPHS ];
#endif /* THOR_LLD_WWDG */

  }    // namespace LookupTables

}    // namespace Thor::LLD::RCC

#endif /* !THOR_HW_RCC_MAPPING_HPP */
