/******************************************************************************
 *  File Name:
 *    hw_spi_driver.cpp
 *
 *  Description:
 *    Driver details for the STM32F4 SPI peripheral
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/spi>
#include <Thor/lld/interface/inc/spi>
#include <Thor/lld/interface/spi/common_driver/spi_common_intf.hpp>


namespace Thor::LLD::SPI
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void prjConfigureTransferWidth( RegisterMap *const periph, const Chimera::SPI::DataSize size )
  {
    // TODO: Does this need to happen on the stm32f4?
  }
}    // namespace Thor::LLD::SPI
