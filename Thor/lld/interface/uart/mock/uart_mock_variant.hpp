/********************************************************************************
 *  File Name:
 *    uart_mock_variant.hpp
 *
 *  Description:
 *    Mock variant of the UART hardware
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_UART_MOCK_VARIANT_HPP
#define THOR_LLD_UART_MOCK_VARIANT_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/interface/uart/uart_intf.hpp>
#include <Thor/lld/interface/uart/uart_types.hpp>

namespace Thor::LLD::UART
{
  /*-------------------------------------------------------------------------------
  Literals
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_UART_PERIPHS = 15;


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
  extern std::array<RegisterMap*, NUM_UART_PERIPHS> PeripheralRegisterMaps;


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeMapping();


}  // namespace Thor::LLD::UART

#endif  /* !THOR_LLD_UART_MOCK_VARIANT_HPP */
