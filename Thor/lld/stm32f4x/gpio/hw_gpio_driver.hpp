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

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/drivers/f4/gpio/hw_gpio_types.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_mapping.hpp>
#include <Thor/drivers/model/gpio_model.hpp>
#include <Thor/drivers/model/interrupt_model.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

namespace Thor::Driver::GPIO
{
  /**
   *  Initialize the low level driver
   *
   *  @return void 
   */
  void initialize();


  class DriverBare : public Model
  {
  public:
    DriverBare();
    ~DriverBare();

    void attach( RegisterMap *const peripheral ) final override;

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
    RegisterMap *periph;
  };


  class DriverThreaded : public Model, public Chimera::Threading::Lockable
  {
  public:
    DriverThreaded();
    ~DriverThreaded();

    void attach( RegisterMap *const peripheral ) final override;

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
    DriverBare bareMetalDriver;
    static constexpr size_t defaultError = 0;
  };

  class DriverAtomic : public Model
  {
  public:
    DriverAtomic();
    ~DriverAtomic();

    void attach( RegisterMap *const peripheral ) final override;

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

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
#endif /* !THOR_HW_DRIVER_GPIO_HPP */