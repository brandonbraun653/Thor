/********************************************************************************
 *   File Name:
 *    event.hpp
 *
 *   Description:
 *    Types used when interacting/responding to peripheral events
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_EVENT_HPP
#define THOR_EVENT_HPP

/* C++ Includes */
#include <cstdint>
#include <vector>

/* Chimera Includes */
#include <Chimera/types/event_types.hpp>

namespace Thor::Event
{
  /**
   *  Processes an event listener that requires an atomic variable to be updated
   *
   *  @param[in]  event       The event that just occurred
   *  @param[in]  listener    The listener to be processed
   *  @param[in]  value       The value to update the atomic with
   *  @return void
   */
  void notifyAtomic( const Chimera::Event::Trigger event, Chimera::Event::Actionable &listener, const uint32_t value );

  /**
   *  Processes an event listener that updates a threading primitive
   *
   *  @param[in]  event       The event that just occurred
   *  @param[in]  listener    The listener to be processed
   *  @return void
   */
  void notifyThread( const Chimera::Event::Trigger event, Chimera::Event::Actionable &listener );

  /**
   *  Processes an event listener that contains a callback function
   *
   *  @param[in]  event       The event that just occurred
   *  @param[in]  listener    The listener to be processed
   *  @param[in]  handle      Some data to be passed into the callback
   *  @param[in]  size        The side of the data that 'handle' points to
   *  @return void
   */
  void executeISRCallback( const Chimera::Event::Trigger event, Chimera::Event::Actionable &listener, void *const handle,
                           const size_t size );

}    // namespace Thor::Event

#endif /* !THOR_DRIVER_PERIPH_EVENT_TYPES_HPP */