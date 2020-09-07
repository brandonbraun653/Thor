/********************************************************************************
 *  File Name:
 *    peripheral_interface.hpp
 *
 *  Description:
 *    Common low level driver peripheral functions. Typically these are helpers
 *    that supply a functionality applied across all peripheral types.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_COMMON_INTERFACE_HPP
#define THOR_LLD_COMMON_INTERFACE_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

namespace Thor::LLD
{
  /**
   *  Looks up an index value that can be used to access distributed resources
   *  associated with a peripheral driver. If that peripheral is not supported
   *  or if the address is invalid, this will return INVALID_RESOURCE_INDEX.
   *
   *  @param[in]  type          Which peripheral type is being queried
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::Peripheral::Type type, const std::uintptr_t address );
}  // namespace Thor::LLD

#endif  /* !THOR_LLD_COMMON_INTERFACE_HPP */
