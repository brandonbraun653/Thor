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
#include <Thor/lld/stm32f4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32f4x/uart/hw_uart_prj.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_prj.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_prj.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32f4x/system/sys_memory_map_prj.hpp>

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
    /**
     *  DMA Peripheral Config Lookup Tables
     */
    extern void DMAInit();
    static constexpr size_t dmaTableSize = Thor::LLD::DMA::NUM_DMA_PERIPHS;

    extern const PCC DMALookup;

    extern RegisterConfig DMA_ClockConfig[ dmaTableSize ];
    extern RegisterConfig DMA_ClockConfigLP[ dmaTableSize ];
    extern RegisterConfig DMA_ResetConfig[ dmaTableSize ];

    extern ClockType_t DMA_SourceClock[ dmaTableSize ];

    /**
     *  GPIO Peripheral Config Lookup Tables
     */
    extern void GPIOInit();
    static constexpr size_t gpioTableSize = Thor::LLD::GPIO::NUM_GPIO_PERIPHS;

    extern const PCC GPIOLookup;

    extern RegisterConfig GPIO_ClockConfig[ gpioTableSize ];
    extern RegisterConfig GPIO_ClockConfigLP[ gpioTableSize ];
    extern RegisterConfig GPIO_ResetConfig[ gpioTableSize ];

    extern ClockType_t GPIO_SourceClock[ gpioTableSize ];

    /**
     *  SPI Peripheral Config Lookup Tables
     */
    extern void SPIInit();
    static constexpr size_t spiTableSize = Thor::LLD::SPI::NUM_SPI_PERIPHS;

    extern const PCC SPILookup;

    extern RegisterConfig SPI_ClockConfig[ spiTableSize ];
    extern RegisterConfig SPI_ClockConfigLP[ spiTableSize ];
    extern RegisterConfig SPI_ResetConfig[ spiTableSize ];

    extern ClockType_t SPI_SourceClock[ spiTableSize ];

    /**
     *  UART Peripheral Config Lookup Tables
     */
    extern void UARTInit();
    static constexpr size_t uartTableSize = Thor::LLD::UART::NUM_UART_PERIPHS;

    extern const PCC UARTLookup;

    extern RegisterConfig UART_ClockConfig[ uartTableSize ];
    extern RegisterConfig UART_ClockConfigLP[ uartTableSize ];
    extern RegisterConfig UART_ResetConfig[ uartTableSize ];

    extern ClockType_t UART_SourceClock[ uartTableSize ];

    /**
     *  USART Peripheral Config Lookup Tables
     */
    extern void USARTInit();
    static constexpr size_t usartTableSize = Thor::LLD::USART::NUM_USART_PERIPHS;

    extern const PCC USARTLookup;

    extern RegisterConfig USART_ClockConfig[ usartTableSize ];
    extern RegisterConfig USART_ClockConfigLP[ usartTableSize ];
    extern RegisterConfig USART_ResetConfig[ usartTableSize ];

    extern ClockType_t USART_SourceClock[ usartTableSize ];

    /**
     *  WWDG Peripheral Config Lookup Tables
     */
    extern void WWDGInit();
    static constexpr size_t wwdgTableSize = Thor::LLD::WWDG::NUM_WWDG_PERIPHS;

    extern const PCC WWDGLookup;

    extern RegisterConfig WWDG_ClockConfig[ wwdgTableSize ];
    extern RegisterConfig WWDG_ClockConfigLP[ wwdgTableSize ];
    extern RegisterConfig WWDG_ResetConfig[ wwdgTableSize ];

    extern ClockType_t WWDG_SourceClock[ wwdgTableSize ];

  }    // namespace LookupTables

}    // namespace Thor::LLD::RCC

#endif /* !THOR_HW_RCC_MAPPING_HPP */
