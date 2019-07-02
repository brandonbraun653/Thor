/********************************************************************************
 *   File Name:
 *    hw_gpio_driver.hpp
 *
 *   Description:
 *    Implements the interface to the STM32F4 series GPIO hardware.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_GPIO_HPP
#define THOR_HW_DRIVER_GPIO_HPP

/* C++ Includes */


/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/gpio_types.hpp>
#include <Chimera/types/threading_types.hpp>

/* Thor Includes */
#include <Thor/drivers/f4/gpio/hw_gpio_types.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_mapping.hpp>
#include <Thor/drivers/model/gpio_model.hpp>
#include <Thor/drivers/model/interrupt_model.hpp>

namespace Thor::Driver::GPIO
{
  class DriverBare : public Model
  {
  public:
    DriverBare();
    ~DriverBare();

    void attach( volatile RegisterMap *const peripheral ) final override;

    void clockEnable() final override;

    void clockDisable() final override;

    Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout ) final override;

    Chimera::Status_t speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout ) final override;

    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout ) final override;

    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout ) final override;

    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout ) final override;

    size_t read( const size_t timeout ) final override;

    size_t driveGet( const uint8_t pin, const size_t timeout ) final override;

    size_t speedGet( const uint8_t pin, const size_t timeout ) final override;

    size_t pullGet( const uint8_t pin, const size_t timeout ) final override;

    size_t alternateFunctionGet( const uint8_t pin, const size_t timeout ) final override;

  private:
    volatile RegisterMap *periph;
    uint8_t accessIndex;
  };


  class DriverThreaded : public Model,
                         public Chimera::Threading::Lockable
  {
  public:
    DriverThreaded();
    ~DriverThreaded();

    void attach( volatile RegisterMap *const peripheral ) final override;

    void clockEnable() final override;

    void clockDisable() final override;

    Chimera::Status_t driveSet(  const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout ) final override;

    Chimera::Status_t speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout ) final override;

    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout ) final override;

    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout ) final override;

    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout ) final override;

    size_t read( const size_t timeout ) final override;

    size_t driveGet( const uint8_t pin, const size_t timeout ) final override;

    size_t speedGet( const uint8_t pin, const size_t timeout ) final override;

    size_t pullGet( const uint8_t pin, const size_t timeout ) final override;

    size_t alternateFunctionGet( const uint8_t pin, const size_t timeout ) final override;

  private:
    DriverBare bareMetalDriver;
    static constexpr size_t defaultError = 0;
  };

  class DriverAtomic : public Model
  {
  public:
    DriverAtomic();
    ~DriverAtomic();

    void attach( volatile RegisterMap *const peripheral ) final override;

    void clockEnable() final override;

    void clockDisable() final override;

    Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout ) final override;

    Chimera::Status_t speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout ) final override;

    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout ) final override;

    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout ) final override;

    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout ) final override;

    size_t read( const size_t timeout ) final override;

    size_t driveGet( const uint8_t pin, const size_t timeout ) final override;

    size_t speedGet( const uint8_t pin, const size_t timeout ) final override;

    size_t pullGet( const uint8_t pin, const size_t timeout ) final override;

    size_t alternateFunctionGet( const uint8_t pin, const size_t timeout ) final override;

  private:
    volatile RegisterMap *periph;
  };

}    // namespace Thor::Driver::GPIO

#endif /* !THOR_HW_DRIVER_GPIO_HPP */