/********************************************************************************
 *  File Name:
 *    gpio_intf.hpp
 *
 *  Description:
 *    STM32 LLD GPIO Interface Spec
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_DRIVER_INTERFACE_HPP
#define THOR_LLD_GPIO_DRIVER_INTERFACE_HPP

/* Chimera Includes */
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in] port         The GPIO port to grab
   *  @param[in] pin          Which pin on the given port
   *  @return bool
   */
  bool isSupported( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

  /**
   *  Gets a raw pointer to the GPIO driver for a particular channel
   *
   *  @note Because GPIO hardware is usually grouped into ports, registers, or banks, the
   *        lookup channel is referencing one of those groupings. On STM32, this typically
   *        means PORTA/B/C/etc. Reference the LLD implementation to figure out which
   *        channel is mapped to which port.
   *
   *  @param[in] port         The GPIO port to grab
   *  @param[in] pin          Which pin on the given port
   *  @return IDriver_sPtr    Instance of the GPIO driver for the requested channel
   */
  Driver_rPtr getDriver( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

  /**
   *  Get's the resource index associated with a particular channel. If not
   *  supported, will return INVALID_RESOURCE_INDEX
   *
   *  @param[in] port         The GPIO port to grab
   *  @param[in] pin          Which pin on the given port
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  class IDriver
  {
  public:
    virtual ~IDriver() = default;

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
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation               |
     *  |:-------------:|:----------------------------------------:|
     *  |            OK | The drive state was successfully updated |
     *  |          FAIL | The drive state failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out              |
     *  | NOT_SUPPORTED | The drive state is not supported         |
     */
    virtual Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive ) = 0;

    /**
     *  Set the drive strength of the GPIO output
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  speed     The drive speed to set
     *  @return Chimera::Status_t
     *
     *  |  Return Value |             Explanation            |
     *  |:-------------:|:----------------------------------:|
     *  |            OK | The speed was successfully updated |
     *  |          FAIL | The speed failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out        |
     *  | NOT_SUPPORTED | The speed is not supported         |
     */
    virtual Chimera::Status_t speedSet( const uint8_t pin, const Thor::LLD::GPIO::Speed speed ) = 0;

    /**
     *  Set the pull up/down resistor configuration
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  pull      The pull up/down state to set
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The pull up/down value was successfully updated |
     *  |          FAIL | The pull up/down value failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out                     |
     *  | NOT_SUPPORTED | The pull up/down value is not supported         |
     */
    virtual Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull ) = 0;

    /**
     *  Writes the entire output data register for the configured port
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  val       The value to set the output register to
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation                |
     *  |:-------------:|:-----------------------------------------:|
     *  |            OK | The output state was successfully updated |
     *  |          FAIL | The output state failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out               |
     *  | NOT_SUPPORTED | The output state is not supported         |
     */
    virtual Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state ) = 0;

    /**
     *  Configures the GPIO alternate function register
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  val       The value to set the AF register to
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The alternate function was successfully updated |
     *  |          FAIL | The alternate function failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out                     |
     *  | NOT_SUPPORTED | The alternate function is not supported         |
     */
    virtual Chimera::Status_t alternateFunctionSet( const uint8_t pin, const Chimera::GPIO::Alternate val ) = 0;

    /**
     *  Reads the given pin's state
     *
     *  @return Chimera::GPIO::State
     */
    virtual Chimera::GPIO::State read( const uint8_t pin ) = 0;

    /**
     *  Reads the drive register for the configured port
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Chimera::GPIO::Drive
     */
    virtual Chimera::GPIO::Drive driveGet( const uint8_t pin ) = 0;

    /**
     *  Reads the speed register for the configured port
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Thor::LLD::GPIO::Speed
     */
    virtual Thor::LLD::GPIO::Speed speedGet( const uint8_t pin ) = 0;

    /**
     *  Reads the pull up/down register for the configured port
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Chimera::GPIO::Pull
     */
    virtual Chimera::GPIO::Pull pullGet( const uint8_t pin ) = 0;

    /**
     *  Reads the current GPIO alternate function register configuration
     *
     *  @param[in]  pin       The pin to act on
     *  @param[in]  timeout   How long to wait for the resource to become available
     *  @return Chimera::GPIO::Alternate
     */
    virtual Chimera::GPIO::Alternate alternateFunctionGet( const uint8_t pin ) = 0;
  };
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_LLD_GPIO_DRIVER_INTERFACE_HPP */
