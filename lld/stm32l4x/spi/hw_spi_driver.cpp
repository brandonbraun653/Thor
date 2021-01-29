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
    /*-------------------------------------------------
    Adjust the FIFO RX threshold based on sizing. If
    SZ == 8Bit, allow RXNE event on 1/4 FIFO lvl, else
    allow RXNE event on 1/2 FIFO lvl. FIFO has 4 lvls.

    See RM 40.4.9
    -------------------------------------------------*/
    if ( size == Chimera::SPI::DataSize::SZ_8BIT )
    {
      /* FIFO must hit 1/4 level before RXNE event */
      FRXTH::set( periph, CR2_FRXTH );
    }
    else
    {
      /* FIFO must hit 1/2 level before RXNE event */
      FRXTH::clear( periph, CR2_FRXTH );
    }
  }
}  // namespace Thor::LLD::SPI
