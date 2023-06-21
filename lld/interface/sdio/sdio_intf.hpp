/******************************************************************************
 *  File Name:
 *    sdio_intf.hpp
 *
 *  Description:
 *    SDIO LLD Interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_SDIO_INTF_HPP
#define THOR_LLD_SDIO_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/sdio>

namespace Thor::LLD::SDIO
{
	/*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
  bool isSupported( const Chimera::SDIO::Channel channel );
}    // namespace Thor::LLD::SDIO

#endif /* !THOR_LLD_SDIO_INTF_HPP */
