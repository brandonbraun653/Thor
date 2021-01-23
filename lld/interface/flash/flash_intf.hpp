/********************************************************************************
 *  File Name:
 *    flash_intf.hpp
 *
 *  Description:
 *    LLD interface description for the flash driver
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_FLASH_INTERFACE_HPP
#define THOR_LLD_FLASH_INTERFACE_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Sets the on-chip flash control access latency
   *
   *  @param[in]  waitStates    Number of states to wait
   *  @return Chimera::Status_t
   */
  Chimera::Status_t setLatency( const size_t waitStates );

}  // namespace Thor::LLD::FLASH

#endif  /* !THOR_LLD_FLASH_INTERFACE_HPP */
