/********************************************************************************
 *  File Name:
 *    timer_intf.hpp
 *
 *  Description:
 *    LLD Timer Interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef LLD_TIMER_INTERFACE_HPP
#define LLD_TIMER_INTERFACE_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/pwm>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::TIMER
{
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
   *  Initializes the low level driver
   */
  Chimera::Status_t initializeModule();

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isChannelSupported( const size_t channel );

  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /**
   * @brief Shared functionality among the General/Basic/Advanced drivers
   */
  class CommonHWDriver
  {
  public:
    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired peripheral
     *  @return void
     */
    Chimera::Status_t attach( RegisterMap *const peripheral );

    /**
     *  Resets the hardware registers back to boot-up values
     *
     *  @return Chimera::Status_t
     */
    Chimera::Status_t reset();

    /**
     *  Enables the peripheral clock
     *
     *  @return void
     */
    void clockEnable();

    /**
     *  Disables the peripheral clock
     *
     *  @return void
     */
    void clockDisable();

    void enableChannel( const size_t channel );

    void disableChannel( const size_t channel );

  protected:
    RegisterMap * mPeriph;
  };

}    // namespace Thor::LLD::TIMER

#endif /* !LLD_TIMER_INTERFACE_HPP */
