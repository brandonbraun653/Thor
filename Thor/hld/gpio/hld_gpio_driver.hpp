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
  /*-------------------------------------------------------------------------------
  Chimera Module Interface Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Chimera::GPIO::IGPIO_sPtr getDriver( const Chimera::GPIO::Port port );

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class Driver : virtual public Chimera::GPIO::IGPIO, public Chimera::Threading::Lockable
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t init( const Chimera::GPIO::PinInit &pinInit ) final override;
    Chimera::Status_t init( const Chimera::GPIO::Port port, const uint8_t pin ) final override;
    Chimera::Status_t setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull ) final override;
    Chimera::Status_t setState( const Chimera::GPIO::State state ) final override;
    Chimera::Status_t getState( Chimera::GPIO::State &state ) final override;
    Chimera::Status_t toggle() final override;

  private:
    uint8_t mChannel;
    Chimera::GPIO::State mLastState;
    Chimera::GPIO::PinInit mInitSettings;
  };
}    // namespace Thor::GPIO

#endif /* THOR_HLD_GPIO_HPP */
