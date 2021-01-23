/********************************************************************************
 *  File Name:
 *    power_intf.hpp
 *
 *  Description:
 *    LLD interface description for the power driver
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_POWER_INTERFACE_HPP
#define THOR_LLD_POWER_INTERFACE_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

namespace Thor::LLD::PWR
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
   *  Sets the internal voltage regulator output voltage
   *
   *  @param[in]  scale         Scaling value to set
   *  @return Chimera::Status_t
   */
  Chimera::Status_t setVoltageScaling( const VoltageScale scale );

}    // namespace Thor::LLD::PWR

#endif /* !THOR_LLD_POWER_INTERFACE_HPP */
