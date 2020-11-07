/********************************************************************************
 *  File Name:
 *    gpio.hpp
 *
 *  Description:
 *    Thor GPIO high level driver
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

/* Thor Includes */
#include <Thor/hld/gpio/hld_gpio_types.hpp>

namespace Thor::GPIO
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );
  Driver_sPtr getDriverShared( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class Driver : public Chimera::Threading::Lockable
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t init( const Chimera::GPIO::PinInit &pinInit );
    Chimera::Status_t init( const Chimera::GPIO::Port port, const uint8_t pin );
    Chimera::Status_t setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull );
    Chimera::Status_t setState( const Chimera::GPIO::State state );
    Chimera::Status_t getState( Chimera::GPIO::State &state );
    Chimera::Status_t toggle();
    Chimera::Status_t attachInterrupt( Chimera::Function::vGeneric &func, const Chimera::EXTI::EdgeTrigger trigger );
    void detachInterrupt();

  private:
    Chimera::GPIO::Alternate mAlternate;
    Chimera::GPIO::Pin mPin;
    Chimera::GPIO::Port mPort;
  };
}    // namespace Thor::GPIO

#endif /* THOR_HLD_GPIO_HPP */
