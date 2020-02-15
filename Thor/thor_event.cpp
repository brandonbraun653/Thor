/********************************************************************************
 *   File Name:
 *    thor_event.cpp
 *
 *   Description:
 *    Implements functions used for event handling
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/callback>
#include <Chimera/thread>

/* Driver Includes */
#include <Thor/event.hpp>


namespace Thor::Event
{
  void notifyAtomic( const Chimera::Event::Trigger event, Chimera::Event::Actionable &listener, const uint32_t value )
  {
    if ( listener.trigger != event )
    {
      return;
    }

    /*------------------------------------------------
    Use get_if() because it doesn't throw an exception on failure
    ------------------------------------------------*/
    const auto atomicNotifier = std::get_if<uint32_t *>( &listener.element );

    /*------------------------------------------------
    Is the variant index pointer and the underlying pointer valid?
    ------------------------------------------------*/
    if ( atomicNotifier && *atomicNotifier )
    {
      **atomicNotifier = value;
    }
  }

  void notifyThread( const Chimera::Event::Trigger event, Chimera::Event::Actionable &listener )
  {
    if ( listener.trigger != event )
    {
      return;
    }

    /*------------------------------------------------
    Use get_if() because it doesn't throw an exception on failure
    ------------------------------------------------*/
    const auto threadNotifier = std::get_if<SemaphoreHandle_t>( &listener.element );

    if ( threadNotifier )
    {
      /*------------------------------------------------
      It's safe to use the ISR variant even if you really aren't inside of one.
      ------------------------------------------------*/
      xSemaphoreGiveFromISR( *threadNotifier, NULL );
    }
  }

  void executeISRCallback( const Chimera::Event::Trigger event, Chimera::Event::Actionable &listener, void *const handle,
                           const size_t size )
  {
    using namespace Chimera::Callback;

    if ( listener.trigger != event )
    {
      return;
    }

    /*------------------------------------------------
    Use get_if() because it doesn't throw an exception on failure
    ------------------------------------------------*/
    const auto callback = std::get_if<ISRCallbackFunction>( &listener.element );

    if ( callback )
    {
      /*------------------------------------------------
      Separate check due to the need to convert to the proper type.
      The compiler does not like *callback(...);
      ------------------------------------------------*/
      const ISRCallbackFunction &tmp = *callback;
      if ( tmp )
      {
        tmp( handle, size );
      }
    }
  }

}    // namespace Thor::Event
