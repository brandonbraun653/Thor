/******************************************************************************
 *  File Name:
 *    sdio_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_SDIO_INTF_DETAIL_HPP
#define THOR_LLD_SDIO_INTF_DETAIL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/sdio/hw_sdio_prj.hpp>
#include <Thor/lld/stm32f4x/sdio/hw_sdio_types.hpp>
#else 
#pragma message( "sdio_detail.hpp: Unknown target for LLD" )
#endif 

#endif  /* !THOR_LLD_SDIO_INTF_DETAIL_HPP */
