/********************************************************************************
 *   File Name:
 *    hw_spi_driver.hpp
 *
 *   Description:
 *    STM32 Driver for the SPI Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_DRIVER_HPP
#define THOR_HW_SPI_DRIVER_HPP

/* C++ Includes */
#include <memory>
#include <vector>

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/model/spi_model.hpp>
#include <Thor/drivers/f4/spi/hw_spi_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  /**
   *  Initializes the low level driver
   *
   *  @return void
   */
  void initialize();


  class Driver : public Thor::Driver::SPI::Model
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attach( RegisterMap *const peripheral ) final override;
    Chimera::Status_t reset() final override;
    void clockEnable() final override;
    void clockDisable() final override;
    size_t getErrorFlags() override;
    size_t getStatusFlags() override;
    Chimera::Status_t configure( const Chimera::SPI::DriverConfig &setup ) final override;
    Chimera::Status_t registerConfig( Chimera::SPI::DriverConfig *config ) final override;
    Chimera::Status_t transfer( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) final override;
    Chimera::Status_t transferIT( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) final override;
    Chimera::Status_t transferDMA( const void *const txBuffer, void *const rxBuffer, const size_t bufferSize ) final override;
    Chimera::Status_t killTransfer() final override;

  private:
    RegisterMap *periph;
    size_t resourceIndex;
    Chimera::SPI::DriverConfig *periphConfig;
  };
}    // namespace Thor::Driver::SPI

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
#endif /* THOR_HW_SPI_DRIVER_HPP */