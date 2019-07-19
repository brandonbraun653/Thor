/********************************************************************************
 *   File Name:
 *    peripheral_event_types.hpp
 *
 *   Description:
 *    Types used when interacting/responding to peripheral events
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_PERIPH_EVENT_TYPES_HPP
#define THOR_DRIVER_PERIPH_EVENT_TYPES_HPP

/* C++ Includes */
#include <vector>

/* Chimera Includes */
#include <Chimera/threading.hpp>

/* Driver Includes */
#include <Thor/drivers/common/types/callback_types.hpp>

namespace Thor::Driver
{
  /**
   *  Generic structure that allows a peripheral to hold different kind of 
   *  notifiers and events 
   */
  class EventResponders
  {
  public:
    std::vector<uint32_t *> atomicNotifier;           /**< Atomically accessible POD */
    std::vector<SemaphoreHandle_t> threadNotifier;    /**< Threading primitive used for signaling */
    std::vector<ConstVoidPtrCallback> callbackAction; /**< Some kind of callable function to execute */

    void clear()
    {
      atomicNotifier.clear();
      threadNotifier.clear();
      callbackAction.clear();
    }

    void notifyAtomic( const Chimera::Event::Trigger event )
    {
      for ( auto &notifier : atomicNotifier )
      {
        if ( notifier )
        {
          *notifier = static_cast<uint32_t>( event );
        }
      }
    }

    void notifyThreaded()
    {
      for ( auto notifier : threadNotifier )
      {
        xSemaphoreGiveFromISR( notifier, nullptr );
      }
    }

    void executeCallbacks( const void *const data )
    {
      for ( auto callback : callbackAction )
      {
        if ( callback )
        {
          callback( data );
        }
      }
    }
  };
}    // namespace Thor::Driver

#endif  /* !THOR_DRIVER_PERIPH_EVENT_TYPES_HPP */