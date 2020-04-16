/********************************************************************************
 *  File Name:
 *    spi_model.hpp
 *
 *  Description:
 *    STM32 Driver SPI Model
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_MODEL_HPP
#define THOR_SPI_MODEL_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/interface/spi/spi_types.hpp>

namespace Thor::LLD::SPI
{
  /**
   *  Initializes the low level driver
   */
  extern Chimera::Status_t initialize();

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel   The channel number to be checked
   *  @return bool
   */
  bool isChannelSupported( const size_t channel );

  class IDriver
  {
  public:
    virtual ~IDriver() = default;

    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired SPI peripheral
     *  @return void
     */
    virtual Chimera::Status_t attach( RegisterMap *const peripheral ) = 0;

    /**
     *  Resets the hardware registers back to boot-up values
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset() = 0;

    /**
     *  Enables the peripheral clock
     *
     *  @return void
     */
    virtual void clockEnable() = 0;

    /**
     *  Disables the peripheral clock
     *
     *  @return void
     */
    virtual void clockDisable() = 0;

    /**
     *  Gets any error flags (bitfield) that might be set
     *
     *  @return ErrorFlags_t
     */
    virtual size_t getErrorFlags() = 0;

    /**
     *  Gets any status flags (bitfield) that might be set
     *
     *  @return StatusFlags_t
     */
    virtual size_t getStatusFlags() = 0;

    virtual Chimera::Status_t configure( const Chimera::SPI::DriverConfig &setup ) = 0;

    virtual Chimera::Status_t registerConfig( Chimera::SPI::DriverConfig *config ) = 0;

    virtual Chimera::Status_t transfer( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) = 0;

    virtual Chimera::Status_t transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) = 0;

    virtual Chimera::Status_t transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) = 0;

    virtual Chimera::Status_t killTransfer() = 0;

    virtual void attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup ) = 0;

    virtual HWTransfer getTransferBlock() = 0;
  };
}    // namespace Thor::LLD::SPI

#endif /* THOR_SPI_MODEL_HPP */