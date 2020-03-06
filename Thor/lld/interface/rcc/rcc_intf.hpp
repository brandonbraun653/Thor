/********************************************************************************
 *  File Name:
 *    rcc_intf.hpp
 *
 *  Description:
 *    STM32 RCC interface modeling for the Thor low level driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_RCC_MODEL_HPP
#define THOR_DRIVER_RCC_MODEL_HPP

/* C++ Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/hld/clock/clock_types.hpp>
#include <Thor/lld/interface/rcc/rcc_types.hpp>

namespace Thor::LLD::RCC
{
  class IClockTree
  {
  public:
    virtual ~IClockTree() = default;

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
     *  @param[in]  periph    The peripheral to modify the clock against
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
     *  Gets the frequency of any major system level clock
     *  
     *  @param[in]  clock     The clock you wish to retrieve the current frequency of
     *  @param[out] freqHz    The current frequency of the requested clock
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t getClockFrequency( const ClockType_t clock, size_t *const freqHz ) = 0;

    /**
     *  Gets the current peripheral clock frequency in Hz
     *
     *  @param[in]  periph    The peripheral type to check
     *  @param[in]  address   Lookup table index tied to the peripheral instance
     *  @param[out] freqHz    The current clock frequency of the requested peripheral
     *  @return size_t
     */
    virtual Chimera::Status_t getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address, size_t *const freqHz ) = 0;
  };

  class IPeripheralController
  {
  public:
    virtual ~IPeripheralController() = default;

    /**
     *  Resets the peripheral using RCC reset registers
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset( const Chimera::Peripheral::Type type, const size_t index ) = 0;

    /**
     *  Enables the peripheral clock
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableClock( const Chimera::Peripheral::Type type, const size_t index ) = 0;

    /**
     *  Disables the peripheral clock
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disableClock( const Chimera::Peripheral::Type type, const size_t index ) = 0;

    /**
     *  Enables the peripheral clock in low power mode
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index ) = 0;

    /**
     *  Disables the peripheral clock in low power mode
     *
     *  @param[in]  address        Indicates which peripheral instance should be accessed
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index ) = 0;
  };


  extern IClockTree *getSystemClockController();

  extern IPeripheralController *getSystemPeripheralController();

}    // namespace Thor::LLD::RCC

#endif /* !THOR_DRIVER_RCC_MODEL_HPP */