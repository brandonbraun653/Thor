/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping_gpio.cpp
 *
 *   Description:
 *    RCC configuration maps for the GPIO peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>


namespace Thor::Driver::RCC::LookupTables
{
/*------------------------------------------------
GPIO Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )
  /**
   *  GPIO clock enable register access lookup table
   */
  const RegisterConfig GPIO_ClockConfig[ gpioTableSize ] = {
    /* GPIOA */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_GPIOAEN },
    /* GPIOB */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_GPIOBEN },
    /* GPIOC */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_GPIOCEN },
    /* GPIOD */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_GPIODEN },
    /* GPIOE */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_GPIOEEN },
    /* GPIOF */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_GPIOFEN },
    /* GPIOG */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_GPIOGEN },
    /* GPIOH */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1ENR_GPIOHEN }
  };

  /**
   *  GPIO low power clock enable register access lookup table
   */
  const RegisterConfig GPIO_ClockConfigLP[ gpioTableSize ] = {
    /* GPIOA */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_GPIOALPEN },
    /* GPIOB */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_GPIOBLPEN },
    /* GPIOC */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_GPIOCLPEN },
    /* GPIOD */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_GPIODLPEN },
    /* GPIOE */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_GPIOELPEN },
    /* GPIOF */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_GPIOFLPEN },
    /* GPIOG */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_GPIOGLPEN },
    /* GPIOH */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1LPENR ) ),
      AHB1LPENR_GPIOHLPEN }
  };

  /**
   *  GPIO reset register access lookup table
   */
  const RegisterConfig GPIO_ResetConfig[ gpioTableSize ] = {
    /* GPIOA */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1RSTR_GPIOARST },
    /* GPIOB */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1RSTR_GPIOBRST },
    /* GPIOC */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1RSTR_GPIOCRST },
    /* GPIOD */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1RSTR_GPIODRST },
    /* GPIOE */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1RSTR_GPIOERST },
    /* GPIOF */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1RSTR_GPIOFRST },
    /* GPIOG */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1RSTR_GPIOGRST },
    /* GPIOH */
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, AHB1ENR ) ),
      AHB1RSTR_GPIOHRST }
  };

  /**
   *  GPIO clocking bus source identifier
   */
  const Configuration::ClockType_t GPIO_SourceClock[ gpioTableSize ] = {
    /* GPIOA */
    Configuration::ClockType::PCLK1,
    /* GPIOB */
    Configuration::ClockType::PCLK1,
    /* GPIOC */
    Configuration::ClockType::PCLK1,
    /* GPIOD */
    Configuration::ClockType::PCLK1,
    /* GPIOE */
    Configuration::ClockType::PCLK1,
    /* GPIOF */
    Configuration::ClockType::PCLK1,
    /* GPIOG */
    Configuration::ClockType::PCLK1,
    /* GPIOH */
    Configuration::ClockType::PCLK1,
  };


  const PCC GPIOLookup = { GPIO_ClockConfig, GPIO_ClockConfigLP, GPIO_ResetConfig, GPIO_SourceClock, gpioTableSize };

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
}    // namespace Thor::Driver::RCC::LookupTables
