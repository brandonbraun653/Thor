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

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  /**
   *  Initializes the low level driver
   *
   *  @return void
   */
  void initialize();


  class Driver
  {
  public:
    Driver();
    ~Driver();

  private:
  };
}    // namespace Thor::Driver::SPI

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
#endif /* THOR_HW_SPI_DRIVER_HPP */