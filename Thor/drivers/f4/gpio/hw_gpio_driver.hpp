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
  class Driver : public Thor::Driver::GPIOModel,
                 public Thor::Driver::SignalModel,
                 public Chimera::Threading::Lockable
  {
  public:
    Driver( RegisterMap *const peripheral );
    ~Driver();

    Chimera::Status_t threadedDriveSet( const Chimera::GPIO::Drive drive ) final override;

    Chimera::Status_t threadedSpeedSet( const Thor::Driver::GPIO::Speed speed ) final override;

    Chimera::Status_t threadedPullSet( const Chimera::GPIO::Pull pull ) final override;

    size_t threadedRead() final override;

    Chimera::Status_t threadedWrite( const size_t val ) final override;

    Chimera::Status_t threadedAlternateFunctionSet( const size_t val ) final override;

    size_t threadedAlternateFunctionGet() final override;

    Chimera::Status_t atomicDriveSet( const Chimera::GPIO::Drive drive ) final override;

    Chimera::Status_t atomicSpeedSet( const Thor::Driver::GPIO::Speed speed ) final override;

    Chimera::Status_t atomicPullSet( const Chimera::GPIO::Pull pull ) final override;

    Chimera::Status_t atomicWrite( const size_t val ) final override;

    size_t atomicRead() final override;

    Chimera::Status_t atomicAlternateFunctionSet( const size_t val ) final override;

    size_t atomicAlternateFunctionGet() final override;

    Chimera::Status_t driveSet( const Chimera::GPIO::Drive drive ) final override;

    Chimera::Status_t speedSet( const Thor::Driver::GPIO::Speed speed ) final override;

    Chimera::Status_t pullSet( const Chimera::GPIO::Pull pull ) final override;

    size_t read() final override;

    Chimera::Status_t write( const size_t val ) final override;

    Chimera::Status_t alternateFunctionSet( const size_t val ) final override;

    size_t alternateFunctionGet() final override;

    Chimera::Status_t enableSignal( const InterruptSignal_t sig ) final override;

    Chimera::Status_t disableSignal( const InterruptSignal_t sig ) final override;

  private:
    RegisterMap *const periph;
    Chimera::Threading::RecursiveMutex_t mutex;
  };

}    // namespace Thor::Driver::GPIO

#endif /* !THOR_HW_DRIVER_GPIO_HPP */