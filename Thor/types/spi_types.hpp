/********************************************************************************
 *   File Name:
 *    spi_types.hpp
 *
 *   Description:
 *    Thor SPI types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_TYPES_HPP
#define THOR_SPI_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <memory>

/* Thor Includes */
#include <Thor/types/gpio_types.hpp>
#include <Thor/types/interrupt_types.hpp>
#include <Thor/types/dma_types.hpp>
#include <Thor/types/clock_types.hpp>

namespace Thor::SPI
{
  class SPIClass;
  using SPIClass_sPtr = std::shared_ptr<SPIClass>;
  using SPIClass_uPtr = std::unique_ptr<SPIClass>;


  struct SPIConfig
  {
    /* IO Config */
    Thor::GPIO::Initializer MOSI;
    Thor::GPIO::Initializer MISO;
    Thor::GPIO::Initializer SCK;
    Thor::GPIO::Initializer NSS;

    /* Peripheral Instance */
    SPI_TypeDef *instance;

    /* Interrupt Settings */
    Thor::Interrupt::Initializer IT_HW;
    Thor::Interrupt::Initializer dmaIT_TX;
    Thor::Interrupt::Initializer dmaIT_RX;

    /* DMA Settings */
    Thor::DMA::Initializer dmaTX;
    Thor::DMA::Initializer dmaRX;

    /* Clock Bus */
    Thor::CLK::ClockBus clockBus;
  };
}    // namespace Thor::SPI
#endif /* !CHIMERA_SPI_TYPES_HPP */