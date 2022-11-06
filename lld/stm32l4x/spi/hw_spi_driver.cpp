/********************************************************************************
 *  File Name:
 *    hw_spi_driver.cpp
 *
 *  Description:
 *    Low level driver interface for specific peripheral functions
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/spi>
#include <Thor/lld/interface/inc/spi>
#include <Thor/lld/interface/spi/common_driver/spi_common_intf.hpp>

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  void prjConfigureTransferWidth( RegisterMap *const periph, const Chimera::SPI::DataSize size )
  {
    /*-------------------------------------------------------------------------
    Adjust the FIFO RX threshold based on sizing. If SZ == 8Bit, allow RXNE
    event on 1/4 FIFO level, else allow RXNE event on 1/2 FIFO lvl. The FIFO
    has four levels.

    See RM 40.4.9
    -------------------------------------------------------------------------*/
    if ( size == Chimera::SPI::DataSize::SZ_8BIT )
    {
      FRXTH::set( periph, CR2_FRXTH );
    }
    else // SZ_16BIT
    {
      FRXTH::clear( periph, CR2_FRXTH );
    }
  }
}  // namespace Thor::LLD::SPI
