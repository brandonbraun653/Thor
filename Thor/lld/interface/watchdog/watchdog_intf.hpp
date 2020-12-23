/********************************************************************************
 *   File Name:
 *    watchdog_model.hpp
 *
 *   Description:
 *    STM32 Driver Model for Generic Watchdogs
 *
 *   2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_WATCHDOG_HPP
#define THOR_DRIVER_MODEL_WATCHDOG_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/watchdog>

/* Thor Includes */
#include <Thor/lld/interface/watchdog/watchdog_types.hpp>

namespace Thor::LLD::Watchdog
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   */
  Chimera::Status_t initializeIWDG();
  Chimera::Status_t initializeWWDG();

  /**
   *  Gets a shared pointer to the USB driver for a particular channel
   *
   *  @param[in] channel        The USB channel to grab (1 indexed)
   *  @return IDriver_sPtr      Instance of the USB driver for the requested channel
   */
  IndependentDriver_rPtr getDriver( const Chimera::Watchdog::IChannel channel );
  WindowDriver_rPtr getDriver( const Chimera::Watchdog::WChannel channel );


  /*-------------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  -------------------------------------------------------------------------------*/
  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isSupported( const Chimera::Watchdog::IChannel channel );
  bool isSupported( const Chimera::Watchdog::WChannel channel );

  /**
   *  Get's the resource index associated with a particular channel. If not
   *  supported, will return INVALID_RESOURCE_INDEX
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::Watchdog::IChannel channel );
  RIndex_t getResourceIndex( const Chimera::Watchdog::WChannel channel );

  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Gets the channel associated with a peripheral address
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return Chimera::USB::Channel
   */
  Chimera::Watchdog::IChannel getIChannel( const std::uintptr_t address );
  Chimera::Watchdog::WChannel getWChannel( const std::uintptr_t address );

  /**
   *  Initializes the drivers by attaching the appropriate peripheral
   *
   *  @param[in]  driverList    List of driver objects to be initialized
   *  @param[in]  numDrivers    How many drivers are in driverList
   *  @return bool
   */
  bool attachDriverInstances( IndependentDriver *const driverList, const size_t numDrivers );
  bool attachDriverInstances( WindowDriver *const driverList, const size_t numDrivers );

  /**
   *  Calculates the best prescale register setting given a desired timeout
   *
   *  @param[in]  ms            Desired watchdog timeout in milliseconds
   *  @param[in]  clock         Frequency of the clock driving the watchdog in Hz
   *  @param[in]  maxCount      Max value the watchdog can count to
   *  @param[in]  actVal        Array of decimal values that are available prescale options
   *  @param[in]  regVal        Array of register settings corresponding to values in "actVal"
   *  @param[in]  len           Number of elements in actVal & regVal
   *  @return uint8_t           Index corresponding to the best prescaler selection in regVal
   */
  uint8_t calculatePrescaler( const size_t ms, const size_t clock, const size_t maxCount, const uint8_t *const actVal,
                              const Reg32_t *const regVal, const size_t len );

  /**
   *  Calculates the best reload register setting given the desired timeout
   *  and the current watchdog peripheral clock prescaler.
   *
   *  @param[in]  ms            Desired watchdog timeout in milliseconds
   *  @param[in]  clock         Frequency of the clock driving the watchdog in Hz
   *  @param[in]  minCount      Min value the watchdog will count to (usually the value when a reset is issued)
   *  @param[in]  maxCount      Max value the watchdog can count to
   *  @param[in]  prescaler     Current prescaler setting (decimal)
   *  @return Reg32_t           Value to be applied to the reload register
   */
  Reg32_t calculateReload( const size_t ms, const size_t clock, const size_t minCount, const size_t maxCount, const size_t prescaler );

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class ICommon
  {
  public:
    virtual ~ICommon() = default;

    /**
     *  Enables the clock that drives the hardware
     *
     *  @return void
     */
    virtual void enableClock() = 0;

    /**
     *  Directly assigns a prescaler value to the hardware register
     *
     *  @see calculatePrescaler()
     *
     *  @param[in]  val           The register value to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setPrescaler( const Reg32_t val ) = 0;

    /**
     *  Directly assigns the value that hardware counter should be
     *  reloaded to upon the watchdog being refreshed.
     *
     *  @see calculateReload()
     *
     *  @param[in]  val           The reload value to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setReload( const Reg32_t val ) = 0;

    /**
     *  Places the watchdog hardware in a state such that it needs to be
     *  periodically refreshed, otherwise a system reset will occur.
     *
     *  @return void
     */
    virtual void start() = 0;

    /**
     *  Refreshes the watchdog counter so that it does not reset the board
     *
     *  @return void
     */
    virtual void reload() = 0;

    /**
     *  Calculates the maximum timeout (mS) that can be achieved with the
     *  given prescaler value.
     *
     *  @param[in]  prescaler     The watchdog clock prescaler (register value)
     *  @return size_t            Millisecond delay
     */
    virtual size_t getMaxTimeout( const Reg32_t prescaler ) = 0;

    /**
     *  Calculates the minimum timeout (mS) that can be achieved with the
     *  given prescaler value.
     *
     *  @param[in]  prescaler     The watchdog clock prescaler (register value)
     *  @return size_t            Millisecond delay
     */
    virtual size_t getMinTimeout( const Reg32_t prescaler ) = 0;

    /**
     *  Gets the currently configured timeout (mS)
     *
     *  @return size_t
     */
    virtual size_t getTimeout() = 0;
  };


  class IIndependent : public virtual ICommon
  {
  public:
    virtual ~IIndependent() = default;

    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired USB peripheral
     *  @return void
     */
    virtual Chimera::Status_t attach( IRegisterMap *const peripheral ) = 0;
  };


  class IWindow : public virtual ICommon
  {
  public:
    virtual ~IWindow() = default;

    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired USB peripheral
     *  @return void
     */
    virtual Chimera::Status_t attach( WRegisterMap *const peripheral ) = 0;

    /**
     *  Calculates the register value needed to properly configure the window in
     *  which a watchdog can be kicked.
     *
     *  @param[in]  ms            The watchdog's timeout in milliseconds
     *  @param[in]  percent       Percentage of the timeout (ms) which defines the window
     *                            that the watchdog can be updated in.
     *  @param[in]  prescaler     The hardware register prescaler value
     *  @return Reg32_t           Register value
     */
    virtual Reg32_t calculateWindow( const size_t ms, const uint8_t percent, const Reg32_t prescaler ) = 0;

    /**
     *  Sets the hardware register watchdog window value
     *
     *  @see calculateWindow()
     *
     *  @param[in]  val           The register value to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setWindow( const Reg32_t val ) = 0;
  };


  class IndependentDriver // ICommon & IIndependent
  {
  public:
    IndependentDriver();
    ~IndependentDriver();

    Chimera::Status_t attach( IRegisterMap *const peripheral );
    void enableClock();
    Chimera::Status_t setPrescaler( const Reg32_t val );
    Chimera::Status_t setReload( const Reg32_t val );
    void start();
    void reload();
    size_t getMaxTimeout( const Reg32_t prescaler );
    size_t getMinTimeout( const Reg32_t prescaler );
    size_t getTimeout();

  private:
    IRegisterMap *mPeriph;
    size_t mResourceIndex;
  };


  class WindowDriver // ICommon & IWindow
  {
  public:
    WindowDriver();
    ~WindowDriver();

    Chimera::Status_t attach( WRegisterMap *const peripheral );
    void enableClock();
    Chimera::Status_t setPrescaler( const Reg32_t val );
    Chimera::Status_t setReload( const Reg32_t val );
    void start();
    void reload();
    size_t getMaxTimeout( const Reg32_t prescaler );
    size_t getMinTimeout( const Reg32_t prescaler );
    size_t getTimeout();

  private:
    IRegisterMap *mPeriph;
    size_t mResourceIndex;
  };

}    // namespace Thor::LLD::Watchdog

#endif /* !THOR_DRIVER_MODEL_WATCHDOG_HPP */
