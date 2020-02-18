/********************************************************************************
 *   File Name:
 *    watchdog_model.hpp
 *
 *   Description:
 *    STM32 Driver Model for Generic Watchdogs
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
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

namespace Thor::Driver::Watchdog
{
  class Basic
  {
  public:
    virtual ~Basic() = default;

    /**
     *  Enables the clock that drives the hardware
     *
     *  @return void
     */
    virtual void enableClock() = 0;

    /**
     *  Calculates the appropriate hardware prescaler given a desired
     *  timeout in milliseconds
     *  
     *  @param[in]  ms            The number of milliseconds to set the timeout to
     *  @return Reg32_t           The hardware register prescaler value
     */
    virtual Reg32_t calculatePrescaler( const size_t ms ) = 0;

    /**
     *  Calculates the appropriate hardware reload value when the counter
     *  is refreshed.
     *
     *  @param[in]  ms            The number of milliseconds to set the timeout to
     *  @param[in]  prescaler     The hardware register prescaler value
     *  @return Reg32_t           The hardware register reload value
     */
    virtual Reg32_t calculateReload( const size_t ms, const Reg32_t prescaler ) = 0;

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


  class Advanced
  {
  public:
    virtual ~Advanced() = default;

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

}    // namespace Thor::Driver::Watchdog


#endif  /* !THOR_DRIVER_MODEL_WATCHDOG_HPP */