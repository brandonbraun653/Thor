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
     *  Sets the closest available timeout that can be achieved
     *  given the current hardware configuration.
     *  
     *  @param[in]  ms            The number of milliseconds to set the timeout to
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t setTimeout( const size_t ms );
    
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
    virtual void kick() = 0;

    /** 
     *  Calculates the maximum delay (mS) that can be achieved with the
     *  given prescaler value.
     *  
     *  @param[in]  prescaler     The watchdog clock prescaler
     *  @return size_t            Millisecond delay
     */
    virtual size_t maxDelay( const uint32_t prescaler ) = 0;

    /** 
     *  Calculates the minimum delay (mS) that can be achieved with the
     *  given prescaler value.
     *  
     *  @param[in]  prescaler     The watchdog clock prescaler
     *  @return size_t            Millisecond delay
     */
    virtual size_t minDelay( const uint32_t prescaler ) = 0;
  };
}    // namespace Thor::Driver::Watchdog


#endif  /* !THOR_DRIVER_MODEL_WATCHDOG_HPP */