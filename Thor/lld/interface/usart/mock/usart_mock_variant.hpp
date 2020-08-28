/********************************************************************************
 *  File Name:
 *    usart_mock_variant.hpp
 *
 *  Description:
 *    Mock variant of the USART hardware
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_USART_MOCK_VARIANT_HPP
#define THOR_LLD_USART_MOCK_VARIANT_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_types.hpp>

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Literals
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_USART_PERIPHS = 15;


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
  extern std::array<RegisterMap*, NUM_USART_PERIPHS> PeripheralRegisterMaps;


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeMapping();
  void initializeRegisters();

}  // namespace Thor::LLD::USART

#endif  /* !THOR_LLD_USART_MOCK_VARIANT_HPP */
