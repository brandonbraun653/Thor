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
#include <Thor/drivers/model/gpio_model.hpp>
#include <Thor/drivers/model/interrupt_model.hpp>

namespace Thor::Driver::GPIO
{
  class DriverBare : public ModelBare
  {
  public:
    DriverBare();
    ~DriverBare();

    void attach( RegisterMap *const peripheral ) final override;

    Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive ) final override;

    Chimera::Status_t speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed ) final override;

    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull ) final override;

    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state ) final override;

    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val ) final override;

    size_t read() final override;

    size_t driveGet( const uint8_t pin ) final override;

    size_t speedGet( const uint8_t pin ) final override;

    size_t pullGet( const uint8_t pin ) final override;

    size_t alternateFunctionGet( const uint8_t pin ) final override;

  private:
    RegisterMap *periph;
  };


  class DriverThreaded : public ModelThreaded,
                         public Chimera::Threading::Lockable
  {
  public:
    DriverThreaded();
    ~DriverThreaded();

    void attach( RegisterMap *const peripheral ) final override;

    Chimera::Status_t driveSet(  const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout ) final override;

    Chimera::Status_t speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout ) final override;

    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout ) final override;

    Chimera::Status_t write( const uint8_t pin, const size_t val, const size_t timeout ) final override;

    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout ) final override;

    size_t read( const size_t timeout ) final override;

    size_t driveGet( const uint8_t pin, const size_t timeout ) final override;

    size_t speedGet( const uint8_t pin, const size_t timeout ) final override;

    size_t pullGet( const uint8_t pin, const size_t timeout ) final override;

    size_t alternateFunctionGet( const uint8_t pin, const size_t timeout ) final override;

  private:
    DriverBare gpio;
    Chimera::Threading::RecursiveMutex_t mutex;
  };

  class DriverAtomic : public ModelAtomic
  {
  public:
    DriverAtomic();
    ~DriverAtomic();

    void attach( RegisterMap *const peripheral ) final override;

    Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive ) final override;

    Chimera::Status_t speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed ) final override;

    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull ) final override;

    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state ) final override;

    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val ) final override;

    size_t read() final override;

    size_t driveGet( const uint8_t pin ) final override;

    size_t speedGet( const uint8_t pin ) final override;

    size_t pullGet( const uint8_t pin ) final override;

    size_t alternateFunctionGet( const uint8_t pin ) final override;

  private:
    RegisterMap *periph;
  };

}    // namespace Thor::Driver::GPIO

#endif /* !THOR_HW_DRIVER_GPIO_HPP */