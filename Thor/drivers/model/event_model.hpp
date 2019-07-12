/********************************************************************************
 *   File Name:
 *    event_model.hpp
 *
 *   Description:
 *    STM32 Driver Event Model
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_CALLBACK_HPP
#define THOR_DRIVER_MODEL_CALLBACK_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/event_types.hpp>

namespace Thor::Driver
{
  class EventListener
  {
  public:
    virtual ~EventListener() = default;

    /**
     *  Registers a listener to a particular event
     *
     *  @param[in]  event      The event type to attach the listener to
     *  @param[in]  listener   The signal to be updated when the event occurs 
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                     Explanation                     |
     *  |:-------------:|:---------------------------------------------------:|
     *  |            OK | Registered the event successfully                   |
     *  |          FAIL | Failed event registration for some reason           |
     *  | NOT_SUPPORTED | This event is not supported by the low level driver |
     */
    virtual Chimera::Status_t registerEventListener( const Chimera::Event::Trigger event, SemaphoreHandle_t *const listener ) = 0;

    /**
     *  Removes a listener from a particular event
     *
     *  @param[in]  event      The event type to remove the listener from
     *  @param[in]  listener   The listener to be removed 
     *
     *  |  Return Value |                     Explanation                     |
     *  |:-------------:|:---------------------------------------------------:|
     *  |            OK | Removed successfully, even if nonexistant           |
     *  | NOT_SUPPORTED | This event is not supported by the low level driver |
     */
    virtual Chimera::Status_t removeEventListener( const Chimera::Event::Trigger event, SemaphoreHandle_t *const listener ) = 0;

  };
}    // namespace Thor::Driver::Serial


#endif /* !THOR_DRIVER_MODEL_CALLBACK_HPP */