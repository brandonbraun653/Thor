/********************************************************************************
 *   File Name:
 *    hw_spi_driver_stm32f4.cpp
 *
 *   Description:
 *    STM32F4 specific driver implementation for SPI
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/chimera.hpp>
#include <Chimera/threading.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/dma.hpp>

#include <Thor/drivers/f4/spi/hw_spi_driver.hpp>
#include <Thor/drivers/f4/spi/hw_spi_mapping.hpp>
#include <Thor/drivers/f4/spi/hw_spi_prj.hpp>
#include <Thor/drivers/f4/spi/hw_spi_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  bool isSPI( const std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
        break;
      }
    }

    return result;
  }

}

#endif  /* TARGET_STM32F4 && THOR_DRIVER_SPI */