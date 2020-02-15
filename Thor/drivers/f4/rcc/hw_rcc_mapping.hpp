/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_RCC_MAPPING_HPP
#define THOR_HW_RCC_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/dma/hw_dma_prj.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_prj.hpp>
#include <Thor/drivers/f4/spi/hw_spi_prj.hpp>
#include <Thor/drivers/f4/uart/hw_uart_prj.hpp>
#include <Thor/drivers/f4/usart/hw_usart_prj.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/system/sys_memory_map_prj.hpp>

namespace Thor::Driver::RCC
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

#if defined( THOR_DRIVER_DMA ) && ( THOR_DRIVER_DMA == 1 )
    static constexpr size_t dmaTableSize = Thor::Driver::DMA::NUM_DMA_PERIPHS;

    extern const PCC DMALookup;

    extern RegisterConfig DMA_ClockConfig[ dmaTableSize ];
    extern RegisterConfig DMA_ClockConfigLP[ dmaTableSize ];
    extern RegisterConfig DMA_ResetConfig[ dmaTableSize ];

    extern Configuration::ClockType_t DMA_SourceClock[ dmaTableSize ];
#endif 

    /**
     *  GPIO Peripheral Config Lookup Tables
     */
    extern void GPIOInit();

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

    static constexpr size_t gpioTableSize = Thor::Driver::GPIO::NUM_GPIO_PERIPHS;

    extern const PCC GPIOLookup;

    extern RegisterConfig GPIO_ClockConfig[ gpioTableSize ];
    extern RegisterConfig GPIO_ClockConfigLP[ gpioTableSize ];
    extern RegisterConfig GPIO_ResetConfig[ gpioTableSize ];

    extern Configuration::ClockType_t GPIO_SourceClock[ gpioTableSize ];
#endif

    /**
     *  SPI Peripheral Config Lookup Tables
     */
    extern void SPIInit();

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

    static constexpr size_t spiTableSize = Thor::Driver::SPI::NUM_SPI_PERIPHS;

    extern const PCC SPILookup;

    extern RegisterConfig SPI_ClockConfig[ spiTableSize ];
    extern RegisterConfig SPI_ClockConfigLP[ spiTableSize ];
    extern RegisterConfig SPI_ResetConfig[ spiTableSize ];

    extern Configuration::ClockType_t SPI_SourceClock[ spiTableSize ];
#endif 

    /**
     *  UART Peripheral Config Lookup Tables
     */
    extern void UARTInit();

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )
    static constexpr size_t uartTableSize = Thor::Driver::UART::NUM_UART_PERIPHS;

    extern const PCC UARTLookup;

    extern RegisterConfig UART_ClockConfig[ uartTableSize ];
    extern RegisterConfig UART_ClockConfigLP[ uartTableSize ];
    extern RegisterConfig UART_ResetConfig[ uartTableSize ];

    extern Configuration::ClockType_t UART_SourceClock[ uartTableSize ];
#endif

    /**
     *  USART Peripheral Config Lookup Tables
     */
    extern void USARTInit();

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )
    static constexpr size_t usartTableSize = Thor::Driver::USART::NUM_USART_PERIPHS;

    extern const PCC USARTLookup;

    extern RegisterConfig USART_ClockConfig[ usartTableSize ];
    extern RegisterConfig USART_ClockConfigLP[ usartTableSize ];
    extern RegisterConfig USART_ResetConfig[ usartTableSize ];

    extern Configuration::ClockType_t USART_SourceClock[ usartTableSize ];
#endif

    /**
     *  WWDG Peripheral Config Lookup Tables
     */
    extern void WWDGInit();

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )
    static constexpr size_t wwdgTableSize = Thor::Driver::WWDG::NUM_WWDG_PERIPHS;

    extern const PCC WWDGLookup;

    extern RegisterConfig WWDG_ClockConfig[ wwdgTableSize ];
    extern RegisterConfig WWDG_ClockConfigLP[ wwdgTableSize ];
    extern RegisterConfig WWDG_ResetConfig[ wwdgTableSize ];

    extern Configuration::ClockType_t WWDG_SourceClock[ wwdgTableSize ];
#endif

  }    // namespace LookupTables

}    // namespace Thor::Driver::RCC

#endif /* !THOR_HW_RCC_MAPPING_HPP */
