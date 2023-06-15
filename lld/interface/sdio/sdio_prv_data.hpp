/******************************************************************************
 *  File Name:
 *    sdio_prv_data.hpp
 *
 *  Description:
 *    Declaration of data the must either be defined by the LLD implementation
 *    or is shared among all possible drivers.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_SDIO_DATA_HPP
#define THOR_LLD_SDIO_DATA_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/cfg>
#include <Thor/lld/interface/sdio/sdio_detail.hpp>
#include <Thor/lld/interface/sdio/sdio_types.hpp>

#if defined( THOR_SDIO )
namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Peripheral Instances
  ---------------------------------------------------------------------------*/
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
  extern RegisterMap *SDIO1_PERIPH;
#endif
}    // namespace Thor::LLD::SDIO
#endif /* THOR_SDIO */

#endif /* !THOR_LLD_SDIO_DATA_HPP */
