/********************************************************************************
 *  File Name:
 *    watchdog_mock_variant.hpp
 *
 *  Description:
 *    Mock variant of the WATCHDOG hardware
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_WATCHDOG_MOCK_VARIANT_HPP
#define THOR_LLD_WATCHDOG_MOCK_VARIANT_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/interface/watchdog/watchdog_types.hpp>

namespace Thor::LLD::WATCHDOG
{
  /*-------------------------------------------------------------------------------
  Literals
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_WATCHDOG_PERIPHS = 15;


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    uint32_t placeholderReg;
  };


  /*-------------------------------------------------------------------------------
  External Variables
  -------------------------------------------------------------------------------*/
  extern std::array<RegisterMap*, NUM_WATCHDOG_PERIPHS> PeripheralRegisterMaps;


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeMapping();
  void initializeRegisters();

}  // namespace Thor::LLD::WATCHDOG

#endif  /* !THOR_LLD_WATCHDOG_MOCK_VARIANT_HPP */
