/********************************************************************************
 *   File Name:
 *    spi_model.hpp
 *
 *   Description:
 *    STM32 Driver SPI Model
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_MODEL_HPP
#define THOR_SPI_MODEL_HPP 

/* Chimera Includes */
#include <Chimera/types/spi_types.hpp>

/* Thor Includes */
//#include <Thor/drivers/common/types/spi_types.hpp>

namespace Thor::Driver::SPI
{
  class Model
  {
  public:
    virtual ~Model() = default;


  };
}    // namespace Thor::Driver::SPI

#endif  /* THOR_SPI_MODEL_HPP */