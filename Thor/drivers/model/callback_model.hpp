/********************************************************************************
 *   File Name:
 *    callback_model.hpp
 *
 *   Description:
 *    STM32 Driver Callback Model
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_CALLBACK_HPP
#define THOR_DRIVER_MODEL_CALLBACK_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Boost Includes */
#include <boost/function.hpp>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* Driver Includes */
#include <Thor/drivers/common/types/callback_types.hpp>

namespace Thor::Driver
{
  class BasicCallback
  {
  public:
    virtual ~BasicCallback() = default;

    /**
     *  Registers a callback to a particular event. This function makes no assumption
     *  on whether or the implementer uses a queue or a single callback.
     *
     *  @param[in]  type      The event type to attach the callback to
     *  @param[in]  onEvent   The function to be called when the event occurs 
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                       Explanation                      |
     *  |:-------------:|:------------------------------------------------------:|
     *  |            OK | Registered the callback successfully                   |
     *  |          FAIL | Failed callback registration for some reason           |
     *  | NOT_SUPPORTED | This function is not supported by the low level driver |
     */
    virtual Chimera::Status_t registerCallback( const CallbackEvent type, const VoidCallback &onEvent ) = 0;
  };
}    // namespace Thor::Driver::Serial


#endif /* !THOR_DRIVER_MODEL_CALLBACK_HPP */