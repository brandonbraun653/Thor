/********************************************************************************
 * File Name:
 *   gpio.hpp
 *
 * Description:
 *   Implements the Thor GPIO driver
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_GPIO_HPP
#define THOR_GPIO_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/interface/gpio_intf.hpp>

/* Thor Includes */
#include <Thor/types/gpio_types.hpp>

namespace Thor::GPIO
{
  class GPIOClass : public Chimera::GPIO::Interface
  {
  public:
    GPIOClass();
    ~GPIOClass();

    Chimera::Status_t init( const Chimera::GPIO::PinInit &pinInit ) final override;

    Chimera::Status_t init( const Chimera::GPIO::Port port, const uint8_t pin ) final override;

    Chimera::Status_t setMode( const Chimera::GPIO::Drive drive, const bool pullup ) final override;

    Chimera::Status_t setState( const Chimera::GPIO::State state ) final override;

    Chimera::Status_t getState( Chimera::GPIO::State &state ) final override;

    Chimera::Status_t toggle() final override;

    /**
     *  A more advanced initialization function that allows full configuration of a pin's behavior in one go
     *
     *  @param[in]  port    The port to use
     *  @param[in]  pin     The pin to use
     *  @param[in]  speed   How "fast" you want the pin to switch. This is effectively drive strength.
     *  @param[in]  alt     Alternate function parameter as defined in the STM32 HAL to remap the GPIO to a peripheral
     *  @return void
     */
    void initAdvanced( const Thor::GPIO::PinPort port, const Thor::GPIO::PinNum pin, const Thor::GPIO::PinSpeed speed,
                       const uint32_t alt );

    static GPIO_InitTypeDef getHALInit( const Thor::GPIO::Initializer &config );

  private:
    bool initialized = false;
    Thor::GPIO::Initializer pinConfig;

    void GPIO_Init( Thor::GPIO::PinPort port, GPIO_InitTypeDef *initStruct );
    void GPIO_ClockEnable( Thor::GPIO::PinPort port );
    void GPIO_ClockDisable( Thor::GPIO::PinPort port );
  };

  extern PinNum convertPinNum( const uint8_t num );
  extern PinPort convertPort( const Chimera::GPIO::Port port );
  extern PinMode convertDrive( const Chimera::GPIO::Drive drive );
  extern PinPull convertPull( const Chimera::GPIO::Pull pull );
  extern Initializer convertPinInit( const Chimera::GPIO::PinInit &pin );
}    // namespace Thor::GPIO


#endif    // !THOR_GPIO_HPP
