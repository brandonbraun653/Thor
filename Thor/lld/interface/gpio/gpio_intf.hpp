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

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr Reg32_t BAD_ALT_FUNC = std::numeric_limits<Reg32_t>::max();

  /*-------------------------------------------------------------------------------
  Public Functions (Implemented by the project)
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Gets a raw pointer to the GPIO driver for a particular channel
   *
   *  @note Because GPIO hardware is usually grouped into ports, registers, or banks, the
   *        lookup channel is referencing one of those groupings. On STM32, this typically
   *        means PORTA/B/C/etc. Reference the LLD implementation to figure out which
   *        channel is mapped to which port.
   *
   *  @param[in]  port        The GPIO port to grab
   *  @param[in]  pin         Which pin on the given port
   *  @return IDriver_sPtr    Instance of the GPIO driver for the requested channel
   */
  Driver_rPtr getDriver( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

  /*-------------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  -------------------------------------------------------------------------------*/
  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  port        The GPIO port to grab
   *  @param[in]  pin         Which pin on the given port
   *  @return bool
   */
  bool isSupported( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

  /**
   *  Gets the resource index associated with a particular port/pin combination.
   *  If not supported, will return INVALID_RESOURCE_INDEX.
   *
   *  This computes a special resource index that is associated with the driver
   *  for each pin.
   *
   *  @param[in]  port        The GPIO port to grab
   *  @param[in]  pin         Which pin on the given port, ranged from [0, DRIVER_MAX_PINS_PER_PORT]
   *  @return RIndex_t
   */
  RIndex_t getPinResourceIndex( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

  /**
   *  Looks up an index value that can be used to access distributed resources
   *  associated with a peripheral instance. If the address is invalid, this will
   *  return INVALID_RESOURCE_INDEX.
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Gets the GPIO port associated with a peripheral address
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return Chimera::GPIO::Port
   */
  Chimera::GPIO::Port getPort( const std::uintptr_t address );

  /**
   *  Gets attributes associated with a particular pin
   *
   *  @param[in]  port    Port to get the attributes for
   *  @param[in]  pin     Pin to get the attributes for
   *  @return PinAttributes*
   */
  const PinAttributes *getPinAttributes( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

  /**
   *  Gets the pin attributes list for all pins associated with a port
   *
   *  @param[in]  port    Port to get the attributes for
   *  @return PortAttributes*
   */
  const PortAttributes* getPortAttributes( const Chimera::GPIO::Port port );

  /**
   *  Initializes the GPIO drivers by attaching the appropriate peripheral
   *
   *  @param[in]  driverList  List of driver objects to be initialized
   *  @param[in]  numDrivers  How many drivers are in driverList
   *  @return bool
   */
  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers );

  /**
   *  Searches through const configuration data to find the alternate function
   *  register configuration value for the given inputs. If any of the inputs
   *  are not supported, will return BAD_ALT_FUNC.
   *
   *  @param[in]  port        The port belonging to the pin
   *  @param[in]  pin         The pin to be reconfigured
   *  @param[in]  alt         The desired alternate function
   *  @return Reg32_t
   */
  Reg32_t findAlternateFunction( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin,
                                 const Chimera::GPIO::Alternate alt );


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Virtual class that defines the expected interface.
  Useful for mocking purposes.
  -------------------------------------------------*/
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


  /*-------------------------------------------------
  Concrete driver declaration. Implements the interface
  of the virtual class, but doesn't inherit due to the
  memory penalties. Definition is done project side.
  -------------------------------------------------*/
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
  };
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_LLD_GPIO_DRIVER_INTERFACE_HPP */
