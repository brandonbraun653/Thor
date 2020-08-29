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
#include <Chimera/clock>
#include <Chimera/common>
#include <Chimera/system>

/* Thor Includes */
#include <Thor/clock>
#include <Thor/lld/interface/rcc/rcc_types.hpp>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class ICoreClock;
  class IPeripheralClock;

  /*-------------------------------------------------------------------------------
  External Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes all system resources to their default state.
   *
   *  @return void
   */
  void initialize();

  /**
   *  Acquires the last known reset event type
   *
   *  @return Chimera::System::ResetEvent
   */
  Chimera::System::ResetEvent getResetReason();

  /**
   *  Clears the previous reset reason from hardware
   *
   *  @return void
   */
  void clearResetReason();

  /**
   *  Gets a reference to the core clock controller instance
   *
   *  @return ICoreClock *
   */
  ICoreClock *getCoreClock();

  /**
   *  Gets a reference to the peripheral clock controller instance
   *
   *  @return IPeripheralClock *
   */
  IPeripheralClock *getPeripheralClock();

  /*-------------------------------------------------------------------------------
  Interface Classes
  -------------------------------------------------------------------------------*/
  class ICoreClock
  {
  public:
    virtual ~ICoreClock() = default;

    /**
     *  Enables the requested clock
     *
     *  @note Assumes that all configuration has been applied
     *
     *  @param[in]  clock   The clock to turn on
     *  @return void
     */
    virtual void enableClock( const Chimera::Clock::Bus clock ) = 0;

    /**
     *  Disables the requested clock
     *
     *  @param[in]  clock   The clock to turn off
     *  @return void
     */
    virtual void disableClock( const Chimera::Clock::Bus clock ) = 0;

    /**
     *  Configures the clock tree according to a user defined method, overriding
     *  other set**() methods in this class.
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t configureProjectClocks() = 0;

    /**
     *  Sets the input clock source used as the system clock
     *
     *  @warning  This assumes that the source has previously been configured
     *            correctly and the clock output has stabilized.
     *
     *  @note After this function is complete, the downstream clocks will need
     *        to be reconfigured
     *
     *  @param[in]  src       The desired source
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setCoreClockSource( const Chimera::Clock::Bus src ) = 0;

    /**
     *  Gets the input clock source used as the system clock
     *
     *  @return Chimera::Clock::Bus
     */
    virtual Chimera::Clock::Bus getCoreClockSource() = 0;

    /**
     *  Sets the frequency of the given clock bus
     *
     *  @param[in]  clock     The desired clock bus to configure
     *  @param[in]  freq      The frequency to set the bus to
     *  @param[in]  enable    Should the clock output be enabled?
     */
    virtual Chimera::Status_t setClockFrequency( const Chimera::Clock::Bus clock, const size_t freq, const bool enable ) = 0;

    /**
     *  Gets the frequency of any major system level clock
     *
     *  @param[in]  clock     The clock you wish to retrieve the current frequency of
     *  @return size_t
     */
    virtual size_t getClockFrequency( const Chimera::Clock::Bus clock ) = 0;

    /**
     *  Gets the current peripheral clock frequency in Hz
     *
     *  @param[in]  periph    The peripheral type to check
     *  @param[in]  address   Base address of the exact peripheral
     *  @return size_t
     */
    virtual size_t getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address ) = 0;
  };

  class IPeripheralClock
  {
  public:
    virtual ~IPeripheralClock() = default;

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

}    // namespace Thor::LLD::RCC

#endif /* !THOR_DRIVER_RCC_MODEL_HPP */
