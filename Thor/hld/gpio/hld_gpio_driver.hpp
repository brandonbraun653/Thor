/********************************************************************************
 *  File Name:
 *    gpio.hpp
 *
 *  Description:
 *    Implements the Thor GPIO driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_GPIO_HPP
#define THOR_HLD_GPIO_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/thread>

namespace Thor::GPIO
{
  /**
   *  Initializes the memory associated with the GPIO HLD
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  High level driver to interact with GPIO. It is naturally thread aware 
   *  but by default won't be thread safe unless configured that way.
   */
  class Driver : virtual public Chimera::GPIO::IGPIO, public Chimera::Threading::Lockable
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t init( const Chimera::GPIO::PinInit &pinInit,
                            const size_t timeout = Chimera::Threading::TIMEOUT_DONT_WAIT ) final override;

    Chimera::Status_t init( const Chimera::GPIO::Port port, const uint8_t pin,
                            const size_t timeout = Chimera::Threading::TIMEOUT_DONT_WAIT ) final override;

    Chimera::Status_t setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull,
                               const size_t timeout = Chimera::Threading::TIMEOUT_DONT_WAIT ) final override;

    Chimera::Status_t setState( const Chimera::GPIO::State state,
                                const size_t timeout = Chimera::Threading::TIMEOUT_DONT_WAIT ) final override;

    Chimera::Status_t getState( Chimera::GPIO::State &state,
                                const size_t timeout = Chimera::Threading::TIMEOUT_DONT_WAIT ) final override;

    Chimera::Status_t toggle( const size_t timeout = Chimera::Threading::TIMEOUT_DONT_WAIT ) final override;

  private:
    uint8_t channel;
    Chimera::GPIO::State lastState;
    Chimera::GPIO::PinInit initSettings;
  };
}    // namespace Thor::GPIO

#endif /* THOR_HLD_GPIO_HPP */
