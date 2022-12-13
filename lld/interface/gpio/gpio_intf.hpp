/******************************************************************************
 *  File Name:
 *    gpio_intf.hpp
 *
 *  Description:
 *    STM32 LLD GPIO Interface Spec
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_DRIVER_INTERFACE_HPP
#define THOR_LLD_GPIO_DRIVER_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/exti>
#include <Chimera/gpio>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <limits>

namespace Thor::LLD::GPIO
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr Reg32_t BAD_ALT_FUNC = std::numeric_limits<Reg32_t>::max();

  /*---------------------------------------------------------------------------
  Public Functions (Implemented by the project)
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes the low level driver in the integrating project
   * @return Chimera::Status_t
   */
  Chimera::Status_t init_prj_driver();

  /*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
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
   *  @return IDriver_rPtr    Instance of the GPIO driver for the requested channel
   */
  Driver_rPtr getLLDriver( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  port        The GPIO port to grab
   *  @param[in]  pin         Which pin on the given port
   *  @return bool
   */
  bool isSupported( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

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
   *  Looks up an index value that can be used to access distributed resources
   *  associated with a peripheral instance. If the port is invalid, this will
   *  return INVALID_RESOURCE_INDEX.
   *
   *  @param[in]  port          Which port to get the resource index for
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::GPIO::Port port );

  /**
   *  Gets the resource index associated with a particular port/pin combination.
   *  If not supported, will return INVALID_RESOURCE_INDEX.
   *
   *  This function adds additional checks to ensure that the exact combination
   *  exists in hardware. The resource index returned access the peripheral driver
   *  for the given hardware pin. Multiple indexes can correspond with a single
   *  peripheral instance in hardware.
   *
   *  @param[in]  port        The GPIO port to grab
   *  @param[in]  pin         Which pin on the given port, ranged from [0, DRIVER_MAX_PINS_PER_PORT]
   *  @return RIndex_t
   */
  RIndex_t getPinResourceIndex( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );

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
  const PortAttributes *getPortAttributes( const Chimera::GPIO::Port port );

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

  /**
   *  Searches through the configuration data to find the EXTI event
   *  line associated with the port/pin configuration.
   *
   *  @param[in]  port        The port to look at
   *  @param[in]  pin         The pin to look at
   *  @return Chimera::EXTI::EventLine_t
   */
  Chimera::EXTI::EventLine_t findEventLine( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin );


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class Driver
  {
  public:
    Driver();
    ~Driver();

    /**
     * @brief Attaches a peripheral instance to the interaction model
     *
     * @param peripheral  Memory mapped struct of the desired GPIO peripheral
     * @return Chimera::Status_t
     */
    Chimera::Status_t attach( RegisterMap *const peripheral );

    /**
     * @brief Enables the peripheral clock
     */
    void clockEnable();

    /**
     * @brief Disables the peripheral clock
     */
    void clockDisable();

    /**
     * @brief Sets the output drive type for the GPIO pin
     *
     * @param pin   The pin to act on
     * @param drive The drive type of the GPIO
     * @return Chimera::Status_t
     */
    Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive );

    /**
     * @brief Set the drive strength of the GPIO output
     *
     * @param pin   The pin to act on
     * @param speed The drive speed to set
     * @return Chimera::Status_t
     */
    Chimera::Status_t speedSet( const uint8_t pin, const Thor::LLD::GPIO::Speed speed );

    /**
     * @brief Set the pull up/down resistor configuration
     *
     * @param pin   The pin to act on
     * @param pull  The pull up/down state to set
     * @return Chimera::Status_t
     */
    Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull );

    /**
     * @brief Writes the pin to a logical state
     *
     * @param pin   The pin to act on
     * @param state The pin state to set
     * @return Chimera::Status_t
     */
    Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state );

    /**
     * @brief Configures the GPIO alternate function register
     *
     * @param pin   The pin to act on
     * @param val   Alternate function to configure
     * @return Chimera::Status_t
     */
    Chimera::Status_t alternateFunctionSet( const uint8_t pin, const Chimera::GPIO::Alternate val );

    /**
     * @brief Reads the given pin's state
     *
     * @param pin   The pin to read
     * @return Chimera::GPIO::State
     */
    Chimera::GPIO::State read( const uint8_t pin );

    /**
     * @brief Gets the currently configured drive setting
     *
     * @param pin   The pin to read settings for
     * @return Chimera::GPIO::Drive
     */
    Chimera::GPIO::Drive driveGet( const uint8_t pin );

    /**
     * @brief Reads the speed register for the configured port
     *
     * @param pin   The pin to act on
     * @return Thor::LLD::GPIO::Speed
     */
    Thor::LLD::GPIO::Speed speedGet( const uint8_t pin );

    /**
     * @brief Reads the pull up/down register for the configured port
     *
     * @param pin   The pin to act on
     * @return Chimera::GPIO::Pull
     */
    Chimera::GPIO::Pull pullGet( const uint8_t pin );

    /**
     * @brief Reads the current GPIO alternate function register configuration
     *
     * @param pin   The pin to act on
     * @return Chimera::GPIO::Alternate
     */
    Chimera::GPIO::Alternate alternateFunctionGet( const uint8_t pin );

    /**
     * @brief Attaches an interrupt callback for a GPIO pin that has been configured
     * to use external interrupts.
     *
     * @param pin     The pin to act on
     * @param func    The function to be called
     * @param trigger What edge to trigger on
     * @return Chimera::Status_t
     */
    Chimera::Status_t attachInterrupt( const uint8_t pin, Chimera::Function::vGeneric &func,
                                       const Chimera::EXTI::EdgeTrigger trigger );

    /**
     * @brief Detaches a previously configured interrupt
     *
     * @param pin   The pin to act on
     */
    void detachInterrupt( const uint8_t pin );

  private:
    RegisterMap *mPeriph;
  };
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_LLD_GPIO_DRIVER_INTERFACE_HPP */
