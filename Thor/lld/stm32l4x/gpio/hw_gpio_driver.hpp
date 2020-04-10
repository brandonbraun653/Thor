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
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/stm32l4x/gpio/hw_gpio_types.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_mapping.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>

namespace Thor::LLD::GPIO
{
  class Driver : public IDriver
  {
  public:
    Driver();
    ~Driver();

    void attach( RegisterMap *const peripheral ) final override;
    void clockEnable() final override;
    void clockDisable() final override;
    Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive ) final override;
    Chimera::Status_t speedSet( const uint8_t pin, const Thor::LLD::GPIO::Speed speed ) final override;
    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull ) final override;
    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state ) final override;
    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const Chimera::GPIO::Alternate val ) final override;
    Chimera::GPIO::State read( const uint8_t pin ) final override;
    Chimera::GPIO::Drive driveGet( const uint8_t pin ) final override;
    Thor::LLD::GPIO::Speed speedGet( const uint8_t pin ) final override;
    Chimera::GPIO::Pull pullGet( const uint8_t pin ) final override;
    Chimera::GPIO::Alternate alternateFunctionGet( const uint8_t pin ) final override;

  private:
    RegisterMap *periph;
  };
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_DRIVER_STM32L4_HPP */
