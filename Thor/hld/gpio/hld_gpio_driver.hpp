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
#ifndef THOR_GPIO_HPP
#define THOR_GPIO_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/hld/gpio/hld_gpio_types.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>

namespace Thor::GPIO
{
  Chimera::Status_t initialize();

  class Driver : virtual public Chimera::GPIO::IGPIO, public Chimera::Threading::Lockable
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t init( const Chimera::GPIO::PinInit &pinInit, const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t init( const Chimera::GPIO::Port port, const uint8_t pin,
                            const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull,
                               const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t setState( const Chimera::GPIO::State state, const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t getState( Chimera::GPIO::State &state, const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t toggle( const size_t timeout = ACCESS_TIMEOUT ) final override;

  private:
    Thor::LLD::GPIO::IGPIO_sPtr lld;
    Chimera::GPIO::PinInit initSettings;
  };
}    // namespace Thor::GPIO

#endif /* THOR_GPIO_HPP */
