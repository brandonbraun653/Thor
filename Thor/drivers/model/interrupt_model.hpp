/********************************************************************************
 *   File Name:
 *    interrupt_model.hpp
 *
 *   Description:
 *    STM32 Driver Interrupt Model
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_INTERRUPT_HPP
#define THOR_DRIVER_MODEL_INTERRUPT_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* Driver Includes */
#include <Thor/drivers/common/types/interrupt_types.hpp>

namespace Thor::Driver
{
  /**
   *  Allows for control over whether or not different asynchronous signals
   *  (interrupts) are enabled or disabled.
   */
  class SignalModel
  {
  public:
    virtual ~SignalModel() = default;

    /**
     *  Enables the interrupt that corresponds with the given signal
     *
     *  @param[in]  sig   The signal to enable an interrupt on
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                       Explanation                       |
     *  |:-------------:|:-------------------------------------------------------:|
     *  |            OK | The interrupt was enabled or already was                |
     *  | NOT_SUPPORTED | This interrupt is not supported by the low level driver |
     */
    virtual Chimera::Status_t enableSignal( const InterruptSignal_t sig ) = 0;

    /**
     *  Disables the interrupt that corresponds with the given signal
     *
     *  @param[in]  sig   The signal to enable an interrupt on
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                       Explanation                       |
     *  |:-------------:|:-------------------------------------------------------:|
     *  |            OK | The interrupt was disabled or already was               |
     *  | NOT_SUPPORTED | This interrupt is not supported by the low level driver |
     */
    virtual Chimera::Status_t disableSignal( const InterruptSignal_t sig ) = 0;

  };
}    // namespace Thor::Driver::Serial


#endif /* !THOR_DRIVER_MODEL_INTERRUPT_HPP */