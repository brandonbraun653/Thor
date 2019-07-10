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

namespace Thor::Serial
{

  namespace STM32HAL
  {
#if ( THOR_STM32HAL_DRIVERS == 1 )
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
#endif /* THOR_STM32HAL_DRIVERS */
  }
}
#endif /* !THOR_SERIAL_TYPES_HPP */