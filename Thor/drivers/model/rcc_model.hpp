/********************************************************************************
 *   File Name:
 *    rcc_model.hpp
 *
 *   Description:
 *    STM32 RCC interface modeling for the Thor driver
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_RCC_MODEL_HPP
#define THOR_DRIVER_RCC_MODEL_HPP

/* C++ Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/gpio_types.hpp>
#include <Chimera/types/peripheral_types.hpp>

/* Thor Includes */
#include <Thor/types/clock_types.hpp>

namespace Thor::Driver::RCC
{
  class ClockTree
  {
  public:
    virtual ~ClockTree() = default;

    /**
     *  Configures the clock tree according to a user defined method, overriding
     *  other set**() methods in this class.
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t configureProjectClocks() = 0;

    /**
     *  Attempts to set the peripheral clock frequency to the given value.
     *  
     *  @note   Does not modify the system core clock in order to accomplish this.
     *  @note   If the frequency cannot be matched, no change will be applied.
     *
     *  @param[in]  periph    The perihperal to modify the clock against
     *  @param[in]  freqHz    The desired frequency to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setPeriphClock( const Chimera::Peripheral::Type periph, const size_t freqHz ) = 0;

    /**
     *  Sets the core clock frequency of the chip. This value will be the clock source 
     *  used to generate all other system clocks.
     *
     *  @param[in]  freqHz    The desired frequency to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setCoreClock( const size_t freqHz ) = 0;

    /**
     *  Sets the input clock source used to generate the system core clock
     *
     *  @param[in]  src       The source used to generate the system core clock
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setCoreClockSource( const Thor::Clock::Source src ) = 0;

    /**
     *  Gets the current system core clock frequency in Hz
     *
     *  @return size_t
     */
    virtual size_t getCoreClock() = 0;

    /**
     *  Gets the clock source used to generate the system core clock
     *
     *  @return Thor::Clock::Source
     */
    virtual Thor::Clock::Source getCoreClockSource() = 0;

    /**
     *  Gets the current peripheral clock frequency in Hz
     *
     *  @param[in]  periph    The peripheral to check
     *  @return size_t
     */
    virtual size_t getPeriphClock( const Chimera::Peripheral::Type periph ) = 0;
  };

}    // namespace Thor::Driver::RCC

#endif /* !THOR_DRIVER_RCC_MODEL_HPP */