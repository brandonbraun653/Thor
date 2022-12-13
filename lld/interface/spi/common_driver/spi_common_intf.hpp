/******************************************************************************
 *  File Name:
 *    spi_common_intf.hpp
 *
 *  Description:
 *    Common interface for low level SPI functions
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_COMMON_INTF_HPP
#define THOR_LLD_SPI_COMMON_INTF_HPP

/* STL Includes */
#include <cstdint>
#include <cstddef>

/* Chimera Includes */
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/lld/interface/spi/spi_detail.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>

namespace Thor::LLD::SPI
{
  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/
  /**
   *  Configures the number of bytes used in the transfer. This is
   *  intended to get around the data packing and FIFO threshold
   *  issues that tend to be present on various peripheral implementations.
   *
   *  @param[in]  periph    The peripheral to act on
   *  @param[in]  size      Transfer width
   *  @return void
   */
  void prjConfigureTransferWidth( RegisterMap *const periph, const Chimera::SPI::DataSize size );
}  // namespace Thor::LLD::SPI

#endif  /* !THOR_LLD_SPI_COMMON_INTF_HPP */
