/********************************************************************************
 *   File Name:
 *    gpio_model.hpp
 *
 *   Description:
 *    STM32 Driver GPIO Model
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_GPIO_MODEL_HPP
#define THOR_GPIO_MODEL_HPP

/* Chimera Includes */
#include <Chimera/gpio>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/interface/gpio/gpio_types.hpp>

namespace Thor::Driver::GPIO
{
  /**
   *  STM32 Custom Thor GPIO Driver that can handle BareMetal, Atomic, and Threaded
   *  access modes.
   *  
   *  @note In non-threaded access modes, the timeout parameter is simply ignored.
   */
  class Model
  {
  public:
    virtual ~Model() = default;

    /**
     *  Attaches a peripheral instance to the interaction model
     *
     *  @param[in]  peripheral    Memory mapped struct of the desired GPIO peripheral
     *  @return void
     */
    virtual void attach( RegisterMap *const peripheral ) = 0;
  
    /**
     *  Enables the peripheral clock
     *  
     *  @return void
     */
    virtual void clockEnable() = 0;

    /**
     *  Disables the peripheral clock
     *
     *  @return void
     */
    virtual void clockDisable() = 0;

    /**
     *  Sets the output drive type for the GPIO pin
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  drive     The drive type of the GPIO
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation               |
     *  |:-------------:|:----------------------------------------:|
     *  |            OK | The drive state was successfully updated |
     *  |          FAIL | The drive state failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out              |
     *  | NOT_SUPPORTED | The drive state is not supported         |
     */
    virtual Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout ) = 0;

    /**
     *  Set the drive strength of the GPIO output
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  speed     The drive speed to set
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Chimera::Status_t
     *
     *  |  Return Value |             Explanation            |
     *  |:-------------:|:----------------------------------:|
     *  |            OK | The speed was successfully updated |
     *  |          FAIL | The speed failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out        |
     *  | NOT_SUPPORTED | The speed is not supported         |
     */
    virtual Chimera::Status_t speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout ) = 0;

    /**
     *  Set the pull up/down resistor configuration
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  pull      The pull up/down state to set
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The pull up/down value was successfully updated |
     *  |          FAIL | The pull up/down value failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out                     |
     *  | NOT_SUPPORTED | The pull up/down value is not supported         |
     */
    virtual Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout ) = 0;

    /**
     *  Writes the entire output data register for the configured port
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  val       The value to set the output register to
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation                |
     *  |:-------------:|:-----------------------------------------:|
     *  |            OK | The output state was successfully updated |
     *  |          FAIL | The output state failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out               |
     *  | NOT_SUPPORTED | The output state is not supported         |
     */
    virtual Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout ) = 0;

    /**
     *  Configures the GPIO alternate function register
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  val       The value to set the AF register to
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The alternate function was successfully updated |
     *  |          FAIL | The alternate function failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out                     |
     *  | NOT_SUPPORTED | The alternate function is not supported         |
     */
    virtual Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout ) = 0;

    /**
     *  Reads the entire input data register for the configured port
     *
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return size_t
     */
    virtual size_t read( const size_t timeout ) = 0;

    /**
     *  Reads the drive register for the configured port
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return size_t
     */
    virtual size_t driveGet( const uint8_t pin, const size_t timeout ) = 0;

    /**
     *  Reads the speed register for the configured port
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return size_t
     */
    virtual size_t speedGet( const uint8_t pin, const size_t timeout ) = 0;

    /**
     *  Reads the pull up/dn register for the configured port
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return size_t
     */
    virtual size_t pullGet( const uint8_t pin, const size_t timeout ) = 0;

    /**
     *  Reads the current GPIO alternate function register configuration
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return size_t
     */
    virtual size_t alternateFunctionGet( const uint8_t pin, const size_t timeout ) = 0;
  };

}    // namespace Thor::Driver::GPIO

#endif /* !THOR_GPIO_MODEL_HPP */