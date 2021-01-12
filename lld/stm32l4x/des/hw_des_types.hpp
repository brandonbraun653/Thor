/********************************************************************************
 *  File Name:
 *    hw_des_types.hpp
 *
 *  Description:
 *    Types used in the LLD DES implementation
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_DES_TYPES_HPP
#define THOR_LLD_DES_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera/Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/des/hw_des_prj.hpp>

namespace Thor::LLD::DES
{
  /**
   *  Memory mapped struct for the Unique Device ID register
   */
  struct UDIDRegisterMap
  {
    volatile Reg32_t UID1;
    volatile Reg32_t UID2;
    volatile Reg32_t UID3;
  };

  static_assert( offsetof( UDIDRegisterMap, UID1 ) == 0x00 );
  static_assert( offsetof( UDIDRegisterMap, UID2 ) == 0x04 );
  static_assert( offsetof( UDIDRegisterMap, UID3 ) == 0x08 );

  REG_ACCESSOR( UDIDRegisterMap, UID1, UDIDR_Msk, UIDField1, BIT_ACCESS_R );
  REG_ACCESSOR( UDIDRegisterMap, UID2, UDIDR_Msk, UIDField2, BIT_ACCESS_R );
  REG_ACCESSOR( UDIDRegisterMap, UID3, UDIDR_Msk, UIDField3, BIT_ACCESS_R );

  /**
   *  Memory mapped struct for the Flash Size data register
   */
  struct FSRegisterMap
  {
    volatile Reg32_t FSDR;
  };
  
  static_assert( offsetof( FSRegisterMap, FSDR ) == 0x00 );
  REG_ACCESSOR( FSRegisterMap, FSDR, FSDR_Msk, FS, BIT_ACCESS_R );
  
  /**
   *  Memory mapped struct for the Package data register
   */
  struct PDRegisterMap
  {
    volatile Reg32_t PDR;
  };

  static_assert( offsetof( PDRegisterMap, PDR ) == 0x00 );
  REG_ACCESSOR( PDRegisterMap, PDR, PDR_Msk, PD, BIT_ACCESS_R );
}

#endif  /* !THOR_LLD_DES_TYPES_HPP */ 
