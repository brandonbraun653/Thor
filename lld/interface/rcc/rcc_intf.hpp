/********************************************************************************
 *  File Name:
 *    rcc_intf.hpp
 *
 *  Description:
 *    STM32 RCC interface modeling for the Thor low level driver. Note that on
 *    some projects, not all of the clock interfaces will be available.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_RCC_INTF_HPP
#define THOR_DRIVER_RCC_INTF_HPP

/* C++ Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/clock>
#include <Chimera/common>
#include <Chimera/system>

/* Thor Includes */
#include <Thor/clock>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/interface/rcc/rcc_types.hpp>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  struct ClockTreeInit;

  /*-------------------------------------------------------------------------------
  Public Functions: Interface Implemented
  -------------------------------------------------------------------------------*/
  /**
   *  Gets the registered PCC structure for a peripheral
   *  @return const PCC *
   */
  PCC LLD_CONST *const getPCCRegistry( const Chimera::Peripheral::Type periph );

  /**
   *  Caches external oscillator frequencies. These are always static due to
   *  being generated from crystals or resonators on the project board.
   *
   *  @param[in]  bus       Which clock bus is being cached
   *  @param[in]  freq      Frequency of the clock
   *  @return void
   */
  void cacheExtOscFreq( const Chimera::Clock::Bus bus, const size_t freq );

  /**
   *  Gets the current frequency of the requested bus in Hz. If the bus is
   *  not enabled or not supported by the device, a value of INVALID_CLOCK
   *  is returned.
   *
   *  @param[in]  bus     Which bus to obtain the frequency for
   *  @return size_t
   */
  size_t getExtOscFreq( const Chimera::Clock::Bus bus );


  /*-------------------------------------------------------------------------------
  Public Functions: Driver Implemented
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes all system resources to their default state
   *  @return void
   */
  void initialize();

  /**
   *  Configures the peripheral registry for the RCC driver. This maps each
   *  supported peripheral type into a set of lookup tables defining clock
   *  register access definitions.
   *
   *  @return void
   */
  void initializeRegistry();

  /**
   *  Acquires the last known reset event type
   *  @return Chimera::System::ResetEvent
   */
  Chimera::System::ResetEvent getResetReason();

  /**
   *  Clears the previous reset reason from hardware
   *  @return void
   */
  void clearResetReason();

  /**
   *  Gets a reference to the core clock controller instance
   *  @return SystemClock *
   */
  SystemClock *getCoreClockCtrl();

  /**
   *  Gets a reference to the peripheral clock controller instance
   *  @return PeripheralController *
   */
  PeripheralController *getPeriphClockCtrl();

  /**
   *  Gets the current frequency of the requested bus in Hz. If the bus is
   *  not enabled or not supported by the device, a value of INVALID_CLOCK
   *  is returned.
   *
   *  @param[in]  bus     Which bus to obtain the frequency for
   *  @return size_t
   */
  size_t getBusFrequency( const Chimera::Clock::Bus bus );

  /**
   *  Conifgures the entire clock tree for the device. This may take a second
   *  due to the likely large number of clocks.
   *
   *  @param[in]  config      The clock tree settings
   *  @return bool            True == Success, False == Failure
   */
  bool configureClockTree( ClockTreeInit &config );

  /**
   *  Helper function to calculate the core PLL oscillator frequency. This clock can then
   *  be divided further to produce the various output clocks on the PLL.
   *
   *  @warning  This calculation assumes that any necessary power configuration for the
   *            desired mode has been performed.
   *
   *  @param[in]  pll         Which PLL on the device to configure
   *  @param[in]  inFreq      Input frequency to the PLL
   *  @param[in]  outFreq     VCO frequency used to drive internal PLL dividers
   *  @param[out] config      Clock config structure to update with the calculated values
   *  @return Chimera::Status_t
   */
  Chimera::Status_t calculatePLLBaseOscillator( const PLLType pll, const size_t inFreq, const size_t outFreq, ClockTreeInit &config );

  /**
   *  Helper function to calculate a PLL output clock register settings.
   *
   *  @warning  This calculation assumes that any necessary power configuration for the
   *            desired mode has been performed.
   *
   *  @param[in]  pll         Which PLL on the device to configure
   *  @param[in]  channel     Output channel the configuration is for
   *  @param[in]  inFreq      Input frequency to the output divisor. This is the PLL core oscillator frequency.
   *  @param[in]  outFreq     Output frequency used to drive circuitry external to PLL
   *  @param[out] config      Clock config structure to update with the calculated values
   *  @return Chimera::Status_t
   */
  Chimera::Status_t calculatePLLOuputOscillator( const PLLType pll, const PLLOut channel, const size_t inFreq, const size_t outFreq, ClockTreeInit &config );


  /*-------------------------------------------------------------------------------
  Interface Classes
  -------------------------------------------------------------------------------*/
  /**
   *  Defines an interface to interact with common STM32 clock bus types.
   *
   *  @warning
   *  Do not inherit from this class, unless the MCU has enough memory to pay for
   *  the VTable penalty. This is mostly for documentation & mocking purposes.
   */
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


  class SystemClock /* Interface of ICoreClock */
  {
  public:
    SystemClock();
    ~SystemClock();
    void enableClock( const Chimera::Clock::Bus clock );
    void disableClock( const Chimera::Clock::Bus clock );
    Chimera::Status_t configureProjectClocks();
    Chimera::Status_t setCoreClockSource( const Chimera::Clock::Bus src );
    Chimera::Clock::Bus getCoreClockSource();
    Chimera::Status_t setClockFrequency( const Chimera::Clock::Bus clock, const size_t freq, const bool enable );
    size_t getClockFrequency( const Chimera::Clock::Bus clock );
    size_t getPeriphClock( const Chimera::Peripheral::Type periph, const std::uintptr_t address );

  private:
    friend SystemClock *getCoreClockCtrl();
  };


  /**
   *  Defines an interface to interrogate and control clocks on peripheral level
   *  of granularity.
   *
   *  @warning
   *  Do not inherit from this class, unless the MCU has enough memory to pay for
   *  the VTable penalty. This is mostly for documentation & mocking purposes.
   */
  class IPeripheralClock
  {
  public:
    virtual ~IPeripheralClock() = default;

    /**
     *  Resets the peripheral using RCC reset registers
     *
     *  @param[in]  address         Indicates which peripheral instance should be accessed
     *  @param[in]  index           Resource index for the peripheral being controlled
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset( const Chimera::Peripheral::Type type, const size_t index ) = 0;

    /**
     *  Enables the peripheral clock
     *
     *  @param[in]  address         Indicates which peripheral instance should be accessed
     *  @param[in]  index           Resource index for the peripheral being controlled
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableClock( const Chimera::Peripheral::Type type, const size_t index ) = 0;

    /**
     *  Disables the peripheral clock
     *
     *  @param[in]  address         Indicates which peripheral instance should be accessed
     *  @param[in]  index           Resource index for the peripheral being controlled
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disableClock( const Chimera::Peripheral::Type type, const size_t index ) = 0;

    /**
     *  Enables the peripheral clock in low power mode
     *
     *  @param[in]  address         Indicates which peripheral instance should be accessed
     *  @param[in]  index           Resource index for the peripheral being controlled
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index ) = 0;

    /**
     *  Disables the peripheral clock in low power mode
     *
     *  @param[in]  address         Indicates which peripheral instance should be accessed
     *  @param[in]  index           Resource index for the peripheral being controlled
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index ) = 0;
  };


  class PeripheralController  /* Interface of IPeripheralClock */
  {
  public:
    PeripheralController();
    ~PeripheralController();

    Chimera::Status_t reset( const Chimera::Peripheral::Type type, const size_t index );
    Chimera::Status_t enableClock( const Chimera::Peripheral::Type type, const size_t index );
    Chimera::Status_t disableClock( const Chimera::Peripheral::Type type, const size_t index );
    Chimera::Status_t enableClockLowPower( const Chimera::Peripheral::Type type, const size_t index );
    Chimera::Status_t disableClockLowPower( const Chimera::Peripheral::Type type, const size_t index );

  private:
    friend PeripheralController *getPeriphClockCtrl();
  };
}    // namespace Thor::LLD::RCC

#endif /* !THOR_DRIVER_RCC_INTF_HPP */
