/******************************************************************************
 *  File Name:
 *    hw_sdio_register_sim.cpp
 *
 *  Description:
 *    Simulator memory backing for the SDIO peripheral
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/sdio/sdio_detail.hpp>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Peripheral Instances
  ---------------------------------------------------------------------------*/
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
  static RegisterMap SDIO1_Data   = { 0 };
  RegisterMap       *SDIO1_PERIPH = &SDIO1_Data;
#endif
}    // namespace Thor::LLD::SDIO
