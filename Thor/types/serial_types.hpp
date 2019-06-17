/********************************************************************************
 *   File Name:
 *    serial_types.hpp
 *
 *   Description:
 *    Thor Serial Types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SERIAL_TYPES_HPP
#define THOR_SERIAL_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/headers.hpp>
#include <Thor/types/interrupt_types.hpp>
#include <Thor/types/dma_types.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
namespace Thor::Serial
{
  /** Allows mapping of either a USART or UART peripheral to the serial class. This is intended to be internal use only. */
  struct HardwareClassMapping
  {
    bool ON_UART;
    uint8_t peripheral_number;
  };

  struct SerialConfig
  {
    /* Peripheral Instance */
    const USART_TypeDef *instance;

    /* Interrupt Settings */
    const Thor::Interrupt::Initializer IT_HW;
    const Thor::Interrupt::Initializer dmaIT_TX;
    const Thor::Interrupt::Initializer dmaIT_RX;

    /* DMA Settings */
    const Thor::DMA::Initializer dmaTX;
    const Thor::DMA::Initializer dmaRX;
  };
}
#endif 

#endif /* !THOR_SERIAL_TYPES_HPP */