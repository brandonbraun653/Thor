/********************************************************************************
 *  File Name:
 *    spi_sim_data.cpp
 *
 *  Description:
 *    Simulator variant data
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/spi/spi_detail.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_SPI )

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
  static RegisterMap SPI1;
  RegisterMap *SPI1_PERIPH = &SPI1;
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
  static RegisterMap SPI2;
  RegisterMap *SPI2_PERIPH = &SPI2;
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
  static RegisterMap SPI3;
  RegisterMap *SPI3_PERIPH = &SPI3;
#endif
}  // namespace Thor::LLD::SPI

#endif
