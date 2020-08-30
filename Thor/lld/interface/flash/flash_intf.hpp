/********************************************************************************
 *  File Name:
 *    flash_intf.hpp
 *
 *  Description:
 *    LLD interface description for the flash driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_FLASH_INTERFACE_HPP
#define THOR_LLD_FLASH_INTERFACE_HPP

/* STL Includes */
#include <cstddef>

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
   *  @return RIndexType
   */
  RIndexType getResourceIndex( const std::uintptr_t address );

}  // namespace Thor::LLD::FLASH

#endif  /* !THOR_LLD_FLASH_INTERFACE_HPP */
