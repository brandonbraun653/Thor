/********************************************************************************
 *   File Name:
 *    spi_types.hpp
 *
 *   Description:
 *    Common SPI types used in Thor drivers
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_DRIVER_SPI_COMMON_TYPES_HPP
#define THOR_DRIVER_SPI_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <memory>

namespace Thor::Driver::SPI
{
  /**
   *  Forward declaration to ease compilation
   */
  struct RegisterMap;

  /**
   *  Control block for an ongoing SPI transfer
   */
  struct HWTransfer
  {
    uint8_t *txBuffer;      /**< Pointer to the SPI TX buffer */
    size_t txTransferCount; /**< TX bytes currently transfered */
    size_t txTransferSize;  /**< TX transfer total byte size */

    uint8_t *rxBuffer;      /**< Pointer to the SPI RX buffer */
    size_t rxTransferCount; /**< TX bytes currently transfered */
    size_t rxTransferSize;  /**< TX transfer total byte size */

    bool waitingOnTX;
    bool waitingOnRX;
    Chimera::Status_t status; /**< Current state of the transfer */
  };


  class Driver;
  using Driver_sPtr = std::shared_ptr<Driver>;
  using Driver_uPtr = std::unique_ptr<Driver>;

  using StatusFlags_t = uint32_t;
  using ErrorFlags_t  = uint32_t;
}

#endif /* !THOR_DRIVER_SPI_COMMON_TYPES_HPP */