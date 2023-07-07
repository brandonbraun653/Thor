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
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/sdio/sdio_types.hpp>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
  /**
   * @brief Checks if the given channel is supported by the hardware
   *
   * @param channel    The channel to check
   * @return bool
   */
  bool isSupported( const Chimera::SDIO::Channel channel );

  /**
   * @brief Looks up a resource index based on a raw peripheral instance
   *
   * @param address       The peripheral address
   * @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   * @brief Gets the channel associated with a peripheral address
   *
   * @param address       Memory address the peripheral is mapped to
   * @return Chimera::Serial::Channel
   */
  Chimera::SDIO::Channel getChannel( const std::uintptr_t address );

  /**
   * @brief Initializes the drivers by attaching the appropriate peripheral
   *
   * @param driverList    List of HW driver objects to be initialized
   * @param numDrivers    How many drivers are in driverList
   * @return bool
   */
  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers );

}    // namespace Thor::LLD::SDIO

#endif /* !THOR_LLD_SDIO_INTF_HPP */
