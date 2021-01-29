/********************************************************************************
 *  File Name:
 *    hw_spi_driver.cpp
 *
 *  Description:
 *    Low level driver interface for specific peripheral functions
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/lld/interface/inc/spi>
#include <Thor/lld/interface/spi/common_driver/spi_common_intf.hpp>

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  void prjConfigureTransferWidth( RegisterMap *const periph, const Chimera::SPI::DataSize size )
  {

  }
}  // namespace Thor::LLD::SPI
