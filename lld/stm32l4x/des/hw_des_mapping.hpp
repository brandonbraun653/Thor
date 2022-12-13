/******************************************************************************
 *  File Name:
 *    hw_des_mapping.hpp
 *
 *  Description:
 *    Defines structures used to map various resources needed in the DES driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_DES_MAPPING_HPP
#define THOR_LLD_DES_MAPPING_HPP

/* Thor Includes */
#include <Thor/lld/stm32l4x/des/hw_des_prj.hpp>
#include <Thor/lld/stm32l4x/des/hw_des_types.hpp>

namespace Thor::LLD::DES
{
  #if defined( STM32_UDID_AVAILABLE )
  extern UDIDRegisterMap * UDIDR;
  #endif

  #if defined( STM32_FS_AVAILABLE )
  extern FSRegisterMap * FSDR;
  #endif

  #if defined( STM32_PD_AVAILABLE )
  extern PDRegisterMap * PDR;
  #endif
}

#endif  /* !THOR_LLD_DES_MAPPING_HPP */
