/********************************************************************************
 *  File Name:
 *    hw_gpio_driver.hpp
 *
 *  Description:
 *    Implements the interface to the STM32F4 series GPIO hardware.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_GPIO_HPP
#define THOR_HW_DRIVER_GPIO_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/stm32f4x/gpio/hw_gpio_types.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_mapping.hpp>
#include <Thor/lld/interface/gpio/gpio_model.hpp>
#include <Thor/lld/interface/interrupt/interrupt_model.hpp>

namespace Thor::LLD::GPIO
{
  /**
   *  Initialize the low level driver
   *
   *  @return void 
   */
  void initialize();

  class Driver : public Model
  {
  public:
    Driver();
    ~Driver();

    void attach( RegisterMap *const peripheral ) final override;

    void clockEnable() final override;

    void clockDisable() final override;

    Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout ) final override;

    Chimera::Status_t speedSet( const uint8_t pin, const Thor::LLD::GPIO::Speed speed, const size_t timeout ) final override;

    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout ) final override;

    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout ) final override;

    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout ) final override;

    size_t read( const size_t timeout ) final override;

    size_t driveGet( const uint8_t pin, const size_t timeout ) final override;

    size_t speedGet( const uint8_t pin, const size_t timeout ) final override;

    size_t pullGet( const uint8_t pin, const size_t timeout ) final override;

    size_t alternateFunctionGet( const uint8_t pin, const size_t timeout ) final override;

  private:
    RegisterMap *periph;
  };
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_DRIVER_GPIO_HPP */