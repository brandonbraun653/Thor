/********************************************************************************
 *  File Name:
 *    spi_types.hpp
 *
 *  Description:
 *    Common LLD SPI types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_COMMON_TYPES_HPP
#define THOR_LLD_SPI_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;

  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr  = Driver *;
  using StatusFlags_t = uint32_t;
  using ErrorFlags_t  = uint32_t;

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
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
}    // namespace Thor::LLD::SPI

#endif /* !THOR_LLD_SPI_COMMON_TYPES_HPP */
