/********************************************************************************
 * File Name:
 *   gpio.hpp
 *
 * Description:
 *   Implements the Thor GPIO driver
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_GPIO_HPP
#define THOR_GPIO_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>
#include <Chimera/interface/gpio_intf.hpp>

/* Thor Includes */
#include <Thor/drivers/gpio.hpp>
#include <Thor/types/gpio_types.hpp>


namespace Thor::GPIO
{
#if ( THOR_DRIVER_GPIO == 1 ) && ( THOR_DRIVER_GPIO == 1 )

  /**
   *  Initialize the GPIO peripheral
   *  
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  class GPIOClass : public Chimera::GPIO::HWInterface
  {
  public:
    GPIOClass();
    ~GPIOClass();

    Chimera::Status_t init( const Chimera::GPIO::PinInit &pinInit, const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t init( const Chimera::GPIO::Port port, const uint8_t pin,
                            const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull,
                               const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t setState( const Chimera::GPIO::State state, const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t getState( Chimera::GPIO::State &state, const size_t timeout = ACCESS_TIMEOUT ) final override;

    Chimera::Status_t toggle( const size_t timeout = ACCESS_TIMEOUT ) final override;

  private:
    Thor::Driver::GPIO::Model *driver;
    Chimera::GPIO::PinInit initSettings;
  };

#endif /* THOR_DRIVER_GPIO */

}    // namespace Thor::GPIO


#endif    // !THOR_GPIO_HPP
