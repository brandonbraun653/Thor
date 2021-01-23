/********************************************************************************
 *  File Name:
 *    flash_types.hpp
 *
 *  Description:
 *    Types and definitions for the embedded FLASH driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_INTF_FLASH_TYPES_HPP
#define THOR_LLD_INTF_FLASH_TYPES_HPP

/* STL Includes */
#include <cstddef>
#include <cstdint>

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  /**
   *  Informs the flash latency configuration method to auto-detect the
   *  best latency settings based on the system voltage and clock configuration.
   */
  static constexpr size_t LATENCY_AUTO_DETECT = 0xACACACAC;


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  /**
   *  Describes the flash latency wait state for a given clock range
   */
  struct WaitStateDesc
  {
    uint16_t minClock;  /**< Minimum clock in MHz */
    uint16_t maxClock;  /**< Maximum clock in MHz */
    uint8_t waitState;  /**< Wait state associated with clock range */
  };

}  // namespace Thor::LLD::FLASH

#endif  /* !THOR_LLD_INTF_FLASH_TYPES_HPP */
