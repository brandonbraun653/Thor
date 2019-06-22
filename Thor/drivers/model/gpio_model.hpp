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
#include <Chimera/threading.hpp>
#include <Chimera/types/gpio_types.hpp>

/* Thor Includes */
#include <Thor/drivers/common/types/gpio_types.hpp>

namespace Thor::Driver
{
  /**
   *  A common GPIO model for all STM32 series devices that Thor supports. It
   *  is intended that each function will directly access the register level
   *  using different guarantees about thread safety and atomicity.
   */
  class GPIOModel
  {
    virtual ~GPIOModel() = default;

    /**
     *  Sets the output drive type for the GPIO pin in a thread safe
     *  manner by using RTOS primitives.
     *
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
    virtual Chimera::Status_t threadedDriveSet( const Chimera::GPIO::Drive drive ) = 0;

    /**
     *  Set the drive strength of the GPIO output
     *
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
    virtual Chimera::Status_t threadedSpeedSet( const Thor::Driver::GPIO::Speed speed ) = 0;

    /**
     *  Set the pull up/down resistor configuration in a thread safe manner
     *
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
    virtual Chimera::Status_t threadedPullSet( const Chimera::GPIO::Pull pull ) = 0;

    /**
     *  Reads the entire input data register for the configured port in a
     *  thread safe manner.
     *
     *  @return size_t
     */
    virtual size_t threadedRead() = 0;

    /**
     *  Writes the entire output data register for the configured port in a
     *  thread safe manner.
     *
     *  @param[in]  val     The value to set the output register to
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation                |
     *  |:-------------:|:-----------------------------------------:|
     *  |            OK | The output state was successfully updated |
     *  |          FAIL | The output state failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out               |
     *  | NOT_SUPPORTED | The output state is not supported         |
     */
    virtual Chimera::Status_t threadedWrite( const size_t val ) = 0;

    /**
     *  Configures the GPIO alternate function register in a thread safe manner
     *
     *  @param[in]  val    The value to set the AF register to
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The alternate function was successfully updated |
     *  |          FAIL | The alternate function failed to be updated     |
     *  |       TIMEOUT | The resource lock timed-out                     |
     *  | NOT_SUPPORTED | The alternate function is not supported         |
     */
    virtual Chimera::Status_t threadedAlternateFunctionSet( const size_t val ) = 0;

    /**
     *  Reads the current GPIO alternate function register configuration in
     *  a thread safe manner.
     *
     *  @return size_t
     */
    virtual size_t threadedAlternateFunctionGet() = 0;

    /**
     *  Atomically sets the output drive by using the bit-band region
     *
     *  @param[in]  drive     The drive type of the GPIO
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation               |
     *  |:-------------:|:----------------------------------------:|
     *  |            OK | The drive state was successfully updated |
     *  |          FAIL | The drive state failed to be updated     |
     *  | NOT_SUPPORTED | The drive state is not supported         |
     */
    virtual Chimera::Status_t atomicDriveSet( const Chimera::GPIO::Drive drive ) = 0;

    /**
     *  Set the drive strength of the GPIO output using the bit-band region
     *
     *  @param[in]  speed     The drive speed to set
     *  @return Chimera::Status_t
     *
     *  |  Return Value |             Explanation            |
     *  |:-------------:|:----------------------------------:|
     *  |            OK | The speed was successfully updated |
     *  |          FAIL | The speed failed to be updated     |
     *  | NOT_SUPPORTED | The speed is not supported         |
     */
    virtual Chimera::Status_t atomicSpeedSet( const Thor::Driver::GPIO::Speed speed ) = 0;

    /**
     *  Set the pull up/down resistor configuration using the bit-band region
     *
     *  @param[in]  pull      The pull up/down state to set
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The pull up/down value was successfully updated |
     *  |          FAIL | The pull up/down value failed to be updated     |
     *  | NOT_SUPPORTED | The pull up/down value is not supported         |
     */
    virtual Chimera::Status_t atomicPullSet( const Chimera::GPIO::Pull pull ) = 0;

    /**
     *  Atomically writes the entire output data register for the configured port
     *
     *  @param[in]  val     The value to set the output register to
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation                |
     *  |:-------------:|:-----------------------------------------:|
     *  |            OK | The output state was successfully updated |
     *  |          FAIL | The output state failed to be updated     |
     *  | NOT_SUPPORTED | The output state is not supported         |
     */
    virtual Chimera::Status_t atomicWrite( const size_t val ) = 0;

    /**
     *  Reads the entire input data register for the configured port using
     *  the bit-band region.
     *
     *  @return size_t
     */
    virtual size_t atomicRead() = 0;

    /**
     *  Configures the GPIO alternate function register using the bit-band region
     *
     *  @param[in]  val    The value to set the AF register to
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The alternate function was successfully updated |
     *  |          FAIL | The alternate function failed to be updated     |
     *  | NOT_SUPPORTED | The alternate function is not supported         |
     */
    virtual Chimera::Status_t atomicAlternateFunctionSet( const size_t val ) = 0;

    /**
     *  Reads the current GPIO alternate function register configuration
     *  using the bit band region.
     *
     *  @return size_t
     */
    virtual size_t atomicAlternateFunctionGet() = 0;

    /**
     *  Set the drive state normally by using a read-modify-write command
     *
     *  @param[in]  drive     The drive type of the GPIO
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation               |
     *  |:-------------:|:----------------------------------------:|
     *  |            OK | The drive state was successfully updated |
     *  |          FAIL | The drive state failed to be updated     |
     *  | NOT_SUPPORTED | The drive state is not supported         |
     */
    virtual Chimera::Status_t driveSet( const Chimera::GPIO::Drive drive ) = 0;

    /**
     *  Set the drive speed normally using a read-modify-write command
     *
     *  @param[in]  speed     The drive speed to set
     *  @return Chimera::Status_t
     *
     *  |  Return Value |             Explanation            |
     *  |:-------------:|:----------------------------------:|
     *  |            OK | The speed was successfully updated |
     *  |          FAIL | The speed failed to be updated     |
     *  | NOT_SUPPORTED | The speed is not supported         |
     */
    virtual Chimera::Status_t speedSet( const Thor::Driver::GPIO::Speed speed ) = 0;

    /**
     *  Set the pull up/down resistor normally using a read-modify-write command
     *
     *  @param[in]  pull      The pull up/down state to set
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The pull up/down value was successfully updated |
     *  |          FAIL | The pull up/down value failed to be updated     |
     *  | NOT_SUPPORTED | The pull up/down value is not supported         |
     */
    virtual Chimera::Status_t pullSet( const Chimera::GPIO::Pull pull ) = 0;

    /**
     *  Reads the entire input data register for the configured port by
     *  simply reading it from the peripheral directly.
     *
     *  @return size_t
     */
    virtual size_t read() = 0;

    /**
     *  Writes the entire output data register for the configured port using
     *  a read-modify-write command.
     *
     *  @param[in]  val     The value to set the output register to
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                Explanation                |
     *  |:-------------:|:-----------------------------------------:|
     *  |            OK | The output state was successfully updated |
     *  |          FAIL | The output state failed to be updated     |
     *  | NOT_SUPPORTED | The output state is not supported         |
     */
    virtual Chimera::Status_t write( const size_t val ) = 0;

    /**
     *  Configures the GPIO alternate function register using the standard
     *  read-modify-write command.
     *
     *  @param[in]  val    The value to set the AF register to
     *  @return Chimera::Status_t
     *
     *  |  Return Value |                   Explanation                   |
     *  |:-------------:|:-----------------------------------------------:|
     *  |            OK | The alternate function was successfully updated |
     *  |          FAIL | The alternate function failed to be updated     |
     *  | NOT_SUPPORTED | The alternate function is not supported         |
     */
    virtual Chimera::Status_t alternateFunctionSet( const size_t val ) = 0;

    /**
     *  Reads the current GPIO alternate function register configuration
     *  using a standard register read.
     *
     *  @return size_t
     */
    virtual size_t alternateFunctionGet() = 0;
  };
}    // namespace Thor::Driver

#endif /* !THOR_GPIO_MODEL_HPP */