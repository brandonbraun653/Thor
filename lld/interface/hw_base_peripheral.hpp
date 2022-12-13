/******************************************************************************
 *  File Name:
 *    hw_base_peripheral.hpp
 *
 *  Description:
 *    Base virtual class that defines common operations that should exist on a
 *    given peripheral driver instance.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_BASE_PERIPHERAL_HPP
#define THOR_LLD_BASE_PERIPHERAL_HPP

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class IBasePeriph
  {
  public:
    virtual ~IBasePeriph() = default;

    /**
     *  Resets the hardware registers back to boot-up values
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset() = 0;

    /**
     *  Reset the peripheral using the RCC driver. Despite the name, this
     *  only affects the ADC peripheral, not the RCC clock configuration.
     *
     *  @return void
     */
    virtual void clockReset() = 0;

    /**
     *  Enables the peripheral clock
     *
     *  @return void
     */
    virtual void clockEnable() = 0;

    /**
     *  Disables the peripheral clock
     *
     *  @return void
     */
    virtual void clockDisable() = 0;

    /**
     *  Disables interrupts only on this peripheral
     *  @return void
     */
    virtual void disableInterrupts() = 0;

    /**
     *  Re-enables interrupts only on this peripheral
     *  @return void
     */
    virtual void enableInterrupts() = 0;
  };
}  // namespace Thor::LLD

#endif  /* !THOR_LLD_BASE_PERIPHERAL_HPP */
