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
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/watchdog_types.hpp>

/* Driver Includes */
//#include <Thor/drivers/common/types/w>

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
     *  @return uint32_t          The hardware register prescaler value
     */
    virtual uint32_t calculatePrescaler( const size_t ms ) = 0;

    /**
     *  Calculates the appropriate hardware reload value when the counter
     *  is refreshed.
     *
     *  @param[in]  ms            The number of milliseconds to set the timeout to
     *  @param[in]  prescaler     The hardware register prescaler value
     *  @return uint32_t          The hardware register reload value
     */
    virtual uint32_t calculateReload( const size_t ms, const uint32_t prescaler ) = 0;

    /**
     *  Directly assigns a prescaler value to the hardware register
     *
     *  @see calculatePrescaler()
     *
     *  @param[in]  val           The register value to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setPrescaler( const uint32_t val ) = 0;

    /** 
     *  Directly assigns the value that hardware counter should be 
     *  reloaded to upon the watchdog being refreshed.
     *
     *  @see calculateReload()
     *  
     *  @param[in]  val           The reload value to be set
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setReload( const uint32_t val ) = 0;

    /**
     *  Places the watchdog hardware in a state such that it needs to be 
     *  periodically refereshed with the kick() function, otherwise causing
     *  a system reset to occur.
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
     *  Calculates the maximum delay (mS) that can be achieved with the
     *  given prescaler value.
     *  
     *  @param[in]  prescaler     The watchdog clock prescaler (register value)
     *  @return size_t            Millisecond delay
     */
    virtual size_t maxDelay( const uint32_t prescaler ) = 0;

    /** 
     *  Calculates the minimum delay (mS) that can be achieved with the
     *  given prescaler value.
     *  
     *  @param[in]  prescaler     The watchdog clock prescaler (register value)
     *  @return size_t            Millisecond delay
     */
    virtual size_t minDelay( const uint32_t prescaler ) = 0;
  };
}    // namespace Thor::Driver::Watchdog


#endif  /* !THOR_DRIVER_MODEL_WATCHDOG_HPP */