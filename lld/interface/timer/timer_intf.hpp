/********************************************************************************
 *  File Name:
 *    timer_intf.hpp
 *
 *  Description:
 *    LLD Timer Interface
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef LLD_TIMER_INTERFACE_HPP
#define LLD_TIMER_INTERFACE_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Driver;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  /*-------------------------------------------------------------------------------
  HLD->LLD Required Free Functions
  -------------------------------------------------------------------------------*/
  size_t millis();
  size_t micros();
  void delayMilliseconds( const size_t ms );
  void delayMicroseconds( const size_t us );
  void blockDelayMillis( const size_t ms );
  void blockDelayMicros( const size_t ms );

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   * @brief Initializes the low level driver
   */
  Chimera::Status_t initializeModule();

  /**
   * @brief Looks up the global resource index based on a raw peripheral address
   *
   * @param address     The peripheral address
   * @return RIndex_t
   */
  RIndex_t getGlobalResourceIndex( const std::uintptr_t address );

  /**
   * @brief Looks up the Timer type resource index based on a raw peripheral address
   *
   * There are a subset of resources per timer type (general, advanced, etc.) that
   * may be accessed.
   *
   * @param address     The peripheral address
   * @return RIndex_t
   */
  RIndex_t getTypeResourceIndex( const std::uintptr_t address );

  /**
   * @brief Translates a peripheral address into the appropriate hardware type
   *
   * @param address     The peripheral address
   * @return HardwareType
   */
  HardwareType getHardwareType( const std::uintptr_t address );

  /**
   * @brief Translates a peripheral address into the appropriate hardware type
   *
   * @param instance    The peripheral instance ID
   * @return HardwareType
   */
  HardwareType getHardwareType( const Chimera::Timer::Instance &instance );

  /**
   * @brief Gets the driver associated with timer instance
   *
   * @param instance    Which instance to get
   * @return Handle_rPtr
   */
  Handle_rPtr getHandle( const Chimera::Timer::Instance &instance );

}    // namespace Thor::LLD::TIMER

#endif /* !LLD_TIMER_INTERFACE_HPP */
