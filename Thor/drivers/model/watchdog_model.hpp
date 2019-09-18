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

    virtual void kick() = 0;

    virtual size_t maxDelay( const uint32_t prescaler ) = 0;

    virtual size_t minDelay( const uint32_t prescaler ) = 0;
  };
}    // namespace Thor::Driver::Watchdog


#endif  /* !THOR_DRIVER_MODEL_WATCHDOG_HPP */