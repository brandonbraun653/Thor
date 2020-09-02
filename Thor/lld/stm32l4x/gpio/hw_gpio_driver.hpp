/********************************************************************************
 *  File Name:
 *    hw_gpio_driver.hpp
 *
 *  Description:
 *    Declares the LLD interface to the STM32L4 series GPIO hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_DRIVER_STM32L4_HPP
#define THOR_HW_GPIO_DRIVER_STM32L4_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/lld/stm32l4x/gpio/hw_gpio_types.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>

namespace Thor::LLD::GPIO
{
  class Driver
  {
  public:
    Driver();
    ~Driver();

    void attach( RegisterMap *const peripheral );
    void clockEnable();
    void clockDisable();
    Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive );
    Chimera::Status_t speedSet( const uint8_t pin, const Thor::LLD::GPIO::Speed speed );
    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull );
    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state );
    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const Chimera::GPIO::Alternate val );
    Chimera::GPIO::State read( const uint8_t pin );
    Chimera::GPIO::Drive driveGet( const uint8_t pin );
    Thor::LLD::GPIO::Speed speedGet( const uint8_t pin );
    Chimera::GPIO::Pull pullGet( const uint8_t pin );
    Chimera::GPIO::Alternate alternateFunctionGet( const uint8_t pin );

  private:
    friend bool attachDriverInstances( Driver *const, const size_t );

    RegisterMap *mPeriph;
    Chimera::GPIO::Port mPort;
    Chimera::GPIO::Pin mPin;
  };
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_DRIVER_STM32L4_HPP */
