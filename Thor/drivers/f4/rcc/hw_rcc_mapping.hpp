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
#include <Chimera/types/peripheral_types.hpp>
#include <Chimera/types/gpio_types.hpp>

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
  namespace LookupTables
  {
    /**
     *  DMA Peripheral Config Lookup Tables
     */
    static constexpr size_t dmaTableSize = Thor::Driver::DMA::NUM_DMA_PERIPHS;

    extern const PCC DMALookup;

    extern const RegisterConfig DMA_ClockConfig[ dmaTableSize ];
    extern const RegisterConfig DMA_ClockConfigLP[ dmaTableSize ];
    extern const RegisterConfig DMA_ResetConfig[ dmaTableSize ];

    extern const Configuration::ClockType_t DMA_SourceClock[ dmaTableSize ];

    /**
     *  GPIO Peripheral Config Lookup Tables
     */
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

    static constexpr size_t gpioTableSize = Thor::Driver::GPIO::NUM_GPIO_PERIPHS;

    extern const PCC GPIOLookup;

    extern const RegisterConfig GPIO_ClockConfig[ gpioTableSize ];
    extern const RegisterConfig GPIO_ClockConfigLP[ gpioTableSize ];
    extern const RegisterConfig GPIO_ResetConfig[ gpioTableSize ];

    extern const Configuration::ClockType_t GPIO_SourceClock[ gpioTableSize ];
#endif

    /**
     *  SPI Peripheral Config Lookup Tables
     */
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

    static constexpr size_t spiTableSize = Thor::Driver::SPI::NUM_SPI_PERIPHS;

    extern const PCC SPILookup;

    extern const RegisterConfig SPI_ClockConfig[ spiTableSize ];
    extern const RegisterConfig SPI_ClockConfigLP[ spiTableSize ];
    extern const RegisterConfig SPI_ResetConfig[ spiTableSize ];

    extern const Configuration::ClockType_t SPI_SourceClock[ spiTableSize ];
#endif 

/**
 *  UART Peripheral Config Lookup Tables
 */
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )
    static constexpr size_t uartTableSize = Thor::Driver::UART::NUM_UART_PERIPHS;

    extern const PCC UARTLookup;

    extern const RegisterConfig UART_ClockConfig[ uartTableSize ];
    extern const RegisterConfig UART_ClockConfigLP[ uartTableSize ];
    extern const RegisterConfig UART_ResetConfig[ uartTableSize ];

    extern const Configuration::ClockType_t UART_SourceClock[ uartTableSize ];
#endif

/**
 *  USART Peripheral Config Lookup Tables
 */
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )
    static constexpr size_t usartTableSize = Thor::Driver::USART::NUM_USART_PERIPHS;

    extern const PCC USARTLookup;

    extern const RegisterConfig USART_ClockConfig[ usartTableSize ];
    extern const RegisterConfig USART_ClockConfigLP[ usartTableSize ];
    extern const RegisterConfig USART_ResetConfig[ usartTableSize ];

    extern const Configuration::ClockType_t USART_SourceClock[ usartTableSize ];
#endif

    /**
     *  WWDG Peripheral Config Lookup Tables
     */
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )
    static constexpr size_t wwdgTableSize = Thor::Driver::WWDG::NUM_WWDG_PERIPHS;

    extern const PCC WWDGLookup;

    extern const RegisterConfig WWDG_ClockConfig[ wwdgTableSize ];
    extern const RegisterConfig WWDG_ClockConfigLP[ wwdgTableSize ];
    extern const RegisterConfig WWDG_ResetConfig[ wwdgTableSize ];

    extern const Configuration::ClockType_t WWDG_SourceClock[ wwdgTableSize ];
#endif

  }    // namespace LookupTables

}    // namespace Thor::Driver::RCC

#endif /* !THOR_HW_RCC_MAPPING_HPP */
