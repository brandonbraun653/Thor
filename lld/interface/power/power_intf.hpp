/******************************************************************************
 *  File Name:
 *    power_intf.hpp
 *
 *  Description:
 *    LLD interface description for the power driver
 *
 *  2020-2024 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_POWER_INTERFACE_HPP
#define THOR_LLD_POWER_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/power/power_detail.hpp>
#include <cstddef>

namespace Thor::LLD::PWR
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class VoltageScale : uint8_t
  {
    SCALE_1,  /**< High power/freq */
    SCALE_2,  /**< Med power/freq */
    SCALE_3,  /**< Low power/freq */

    NUM_OPTIONS,
    INVALID
  };


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Enables the peripheral clock for the power module
   *  @return void
   */
  void clockEnable();

  /**
   *  Sets the internal voltage regulator output voltage
   *
   *  @param[in]  scale         Scaling value to set
   *  @return Chimera::Status_t
   */
  Chimera::Status_t setVoltageScaling( const VoltageScale scale );

  /**
   *  Gets the currently configured voltage scaling
   *  @return VoltageScale
   */
  VoltageScale getVoltageScale();

  /**
   *  Enables/disables voltage regulators and switches needed to overdrive
   *  the core clock.
   *
   *  @param[in]  state         Enable/disable
   *  @return void
   */
  void setOverdriveMode( const bool state );

}    // namespace Thor::LLD::PWR

#endif /* !THOR_LLD_POWER_INTERFACE_HPP */
