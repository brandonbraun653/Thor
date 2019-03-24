#pragma once
#ifndef THOR_GPIO_H_
#define THOR_GPIO_H_

/* C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <string>

/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/move/unique_ptr.hpp>

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/utilities.hpp>

/* Chimera Includes */
#if defined( USING_CHIMERA )
#include <Chimera/interface.hpp>
#endif

namespace Thor
{
  namespace Peripheral
  {
    namespace GPIO
    {
      class GPIOClass;
      class ChimeraGPIO;

      typedef boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> GPIOClass_sPtr;
      typedef boost::movelib::unique_ptr<Thor::Peripheral::GPIO::GPIOClass> GPIOClass_uPtr;

      class GPIOClass
      {
        friend class ChimeraGPIO;

      public:
        /**
         *   A basic initialization function to open a port and pin with default settings
         *
         *   @param[in]  port    The port to use
         *   @param[in]  pin     The pin to use
         *   @return void
         */
        void init( const Thor::Definitions::GPIO::PinPort port, const Thor::Definitions::GPIO::PinNum pin );

        /**
         *   A more advanced initialization function that allows full configuration of a pin's behavior
         *
         *   @param[in]  port    The port to use
         *   @param[in]  pin     The pin to use
         *   @param[in]  speed   How "fast" you want the pin to switch. This is effectively drive strength.
         *   @param[in]  alt     Alternate function parameter as defined in the STM32 HAL to remap the GPIO to a peripheral
         *   @return void
         */
        void initAdvanced( const Thor::Definitions::GPIO::PinPort port, const Thor::Definitions::GPIO::PinNum pin,
                           const Thor::Definitions::GPIO::PinSpeed speed = Thor::Definitions::GPIO::PinSpeed::HIGH_SPD,
                           const uint32_t alt                            = Thor::Definitions::GPIO::NOALTERNATE );

        /**
         *   Set the pin mode and pull up/down resistor behavior
         *
         *   @param[in]  mode    The mode to drive the pin as
         *   @param[in]  pull    Pin pullup/dn resistor behavior
         *   @return void
         */
        void mode( const Thor::Definitions::GPIO::PinMode mode, const Thor::Definitions::GPIO::PinPull pull );

        /**
         *   Set the pin to a given logic level
         *
         *   @param[in]  state    Logic level to drive the pin to
         *   @return void
         */
        void write( const Thor::Definitions::GPIO::LogicLevel state );

        /**
         *   Toggles the logic level of the pin
         *
         *   @return void
         */
        void toggle();

        /**
         *   Returns back the logic level of the pin
         *
         *   @return true if high, false if low
         */
        bool read();

        GPIOClass()  = default;
        ~GPIOClass() = default;

        static GPIO_InitTypeDef getHALInit( const Thor::Definitions::GPIO::PinConfig &config );

      private:
        Thor::Definitions::GPIO::PinConfig pinConfig;

        void GPIO_Init( Thor::Definitions::GPIO::PinPort port, GPIO_InitTypeDef *initStruct );
        void GPIO_ClockEnable( Thor::Definitions::GPIO::PinPort port );
        void GPIO_ClockDisable( Thor::Definitions::GPIO::PinPort port );
      };


#if defined( USING_CHIMERA )
      class ChimeraGPIO : public Chimera::GPIO::Interface
      {
      public:
        Chimera::GPIO::Status init( const Chimera::GPIO::Port port, const uint8_t pin ) override;

        Chimera::GPIO::Status setMode( const Chimera::GPIO::Drive drive, const bool pullup ) override;

        Chimera::GPIO::Status setState( const Chimera::GPIO::State state ) override;

        Chimera::GPIO::Status getState( Chimera::GPIO::State &state ) override;

        Chimera::GPIO::Status toggle() override;

        static const Thor::Definitions::GPIO::PinNum convertPinNum( const uint8_t num );
        static const Thor::Definitions::GPIO::PinPort convertPort( const Chimera::GPIO::Port port );
        static const Thor::Definitions::GPIO::PinMode convertDrive( const Chimera::GPIO::Drive drive );
        static const Thor::Definitions::GPIO::PinPull convertPull( const Chimera::GPIO::Pull pull );
        static const Thor::Definitions::GPIO::PinConfig convertPinInit( const Chimera::GPIO::PinInit &pin );

      private:
        Thor::Peripheral::GPIO::GPIOClass gpioPin = Thor::Peripheral::GPIO::GPIOClass();
      };

#endif   /* !USING_CHIMERA */
    }    // namespace GPIO
  }      // namespace Peripheral
}    // namespace Thor


#endif    // !_GPIO_H_
