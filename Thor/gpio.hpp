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
#ifndef THOR_GPIO_H_
#define THOR_GPIO_H_

/* C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <string>

/* Chimera Includes */
#include <Chimera/interface.hpp>

/* Thor Includes */
#include <Thor/config.hpp>
#include <Thor/definitions.hpp>
#include <Thor/utilities.hpp>

namespace Thor
{
  namespace GPIO
  {
    class GPIOClass;

    using GPIOClass_sPtr = std::shared_ptr<GPIOClass>;
    using GPIOClass_uPtr = std::unique_ptr<GPIOClass>;

    class GPIOClass : public Chimera::GPIO::Interface
    {
    public:
      GPIOClass()  = default;
      ~GPIOClass() = default;

      Chimera::Status_t init( const Chimera::GPIO::Port port, const uint8_t pin ) final override;

      Chimera::Status_t setMode( const Chimera::GPIO::Drive drive, const bool pullup ) final override;

      Chimera::Status_t setState( const Chimera::GPIO::State state ) final override;

      Chimera::Status_t getState( Chimera::GPIO::State &state ) final override;

      Chimera::Status_t toggle() final override;

      static GPIO_InitTypeDef getHALInit( const Thor::GPIO::PinConfig &config );

    private:
      Thor::GPIO::PinConfig pinConfig;

      void GPIO_Init( Thor::GPIO::PinPort port, GPIO_InitTypeDef *initStruct );
      void GPIO_ClockEnable( Thor::GPIO::PinPort port );
      void GPIO_ClockDisable( Thor::GPIO::PinPort port );
    };
  }    // namespace GPIO
}    // namespace Thor


#endif    // !_GPIO_H_
