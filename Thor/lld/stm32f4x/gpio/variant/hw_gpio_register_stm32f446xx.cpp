/********************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32f446xx.cpp
 *
 *  Description:
 *    Explicit STM32F446xx GPIO data and routines
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/lld/stm32f4x/gpio/hw_gpio_driver.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_mapping.hpp>
#include <Thor/lld/stm32f4x/gpio/variant/hw_gpio_register_stm32f446xx.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32F446xx ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
#if defined( EMBEDDED )
  RegisterMap *GPIOA_PERIPH = reinterpret_cast<RegisterMap *>( GPIOA_BASE_ADDR );
  RegisterMap *GPIOB_PERIPH = reinterpret_cast<RegisterMap *>( GPIOB_BASE_ADDR );
  RegisterMap *GPIOC_PERIPH = reinterpret_cast<RegisterMap *>( GPIOC_BASE_ADDR );
  RegisterMap *GPIOD_PERIPH = reinterpret_cast<RegisterMap *>( GPIOD_BASE_ADDR );
  RegisterMap *GPIOE_PERIPH = reinterpret_cast<RegisterMap *>( GPIOE_BASE_ADDR );
  RegisterMap *GPIOF_PERIPH = reinterpret_cast<RegisterMap *>( GPIOF_BASE_ADDR );
  RegisterMap *GPIOG_PERIPH = reinterpret_cast<RegisterMap *>( GPIOG_BASE_ADDR );
  RegisterMap *GPIOH_PERIPH = reinterpret_cast<RegisterMap *>( GPIOH_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, Chimera::GPIO::Port> InstanceToPortMap{
    { reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), Chimera::GPIO::Port::PORTA },
    { reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), Chimera::GPIO::Port::PORTB },
    { reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), Chimera::GPIO::Port::PORTC },
    { reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), Chimera::GPIO::Port::PORTD },
    { reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), Chimera::GPIO::Port::PORTE },
    { reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ), Chimera::GPIO::Port::PORTF },
    { reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ), Chimera::GPIO::Port::PORTG },
    { reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), Chimera::GPIO::Port::PORTH }
  };

  Chimera::Container::LightFlatMap<Chimera::GPIO::Port, decltype( GPIOA_PERIPH )> PortToInstanceMap{
    { Chimera::GPIO::Port::PORTA, GPIOA_PERIPH }, { Chimera::GPIO::Port::PORTB, GPIOB_PERIPH },
    { Chimera::GPIO::Port::PORTC, GPIOC_PERIPH }, { Chimera::GPIO::Port::PORTD, GPIOD_PERIPH },
    { Chimera::GPIO::Port::PORTE, GPIOE_PERIPH }, { Chimera::GPIO::Port::PORTF, GPIOF_PERIPH },
    { Chimera::GPIO::Port::PORTG, GPIOG_PERIPH }, { Chimera::GPIO::Port::PORTH, GPIOH_PERIPH }
  };

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), GPIOA_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), GPIOB_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), GPIOC_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), GPIOD_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), GPIOE_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ), GPIOF_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ), GPIOG_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), GPIOH_RESOURCE_INDEX }
  };

#elif defined( _SIM )
  RegisterMap *GPIOA_PERIPH = nullptr;
  RegisterMap *GPIOB_PERIPH = nullptr;
  RegisterMap *GPIOC_PERIPH = nullptr;
  RegisterMap *GPIOD_PERIPH = nullptr;
  RegisterMap *GPIOE_PERIPH = nullptr;
  RegisterMap *GPIOF_PERIPH = nullptr;
  RegisterMap *GPIOG_PERIPH = nullptr;
  RegisterMap *GPIOH_PERIPH = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, Chimera::GPIO::Port> InstanceToPortMap;
  Chimera::Container::LightFlatMap<Chimera::GPIO::Port, RegisterMap *> PortToInstanceMap;
  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
#endif

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    GPIOA_PERIPH = new RegisterMap;
    GPIOB_PERIPH = new RegisterMap;
    GPIOC_PERIPH = new RegisterMap;
    GPIOD_PERIPH = new RegisterMap;
    GPIOE_PERIPH = new RegisterMap;
    GPIOF_PERIPH = new RegisterMap;
    GPIOG_PERIPH = new RegisterMap;
    GPIOH_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ GPIOA_RESOURCE_INDEX ] = GPIOA_PERIPH;
    PeripheralList[ GPIOB_RESOURCE_INDEX ] = GPIOB_PERIPH;
    PeripheralList[ GPIOC_RESOURCE_INDEX ] = GPIOC_PERIPH;
    PeripheralList[ GPIOD_RESOURCE_INDEX ] = GPIOD_PERIPH;
    PeripheralList[ GPIOE_RESOURCE_INDEX ] = GPIOE_PERIPH;
    PeripheralList[ GPIOF_RESOURCE_INDEX ] = GPIOF_PERIPH;
    PeripheralList[ GPIOG_RESOURCE_INDEX ] = GPIOG_PERIPH;
    PeripheralList[ GPIOH_RESOURCE_INDEX ] = GPIOH_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), Chimera::GPIO::Port::PORTA );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), Chimera::GPIO::Port::PORTB );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), Chimera::GPIO::Port::PORTC );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), Chimera::GPIO::Port::PORTD );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), Chimera::GPIO::Port::PORTE );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ), Chimera::GPIO::Port::PORTF );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ), Chimera::GPIO::Port::PORTG );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), Chimera::GPIO::Port::PORTH );


    PortToInstanceMap.append( Chimera::GPIO::Port::PORTA, GPIOA_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTB, GPIOB_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTC, GPIOC_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTD, GPIOD_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTE, GPIOE_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTF, GPIOF_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTG, GPIOG_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTH, GPIOH_PERIPH );


    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), GPIOA_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), GPIOB_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), GPIOC_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), GPIOD_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), GPIOE_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ), GPIOF_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ), GPIOG_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), GPIOH_RESOURCE_INDEX );

#endif
  }
}    // namespace Thor::LLD::GPIO

namespace Thor::Driver::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_gpio_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig GPIO_ClockConfig[ gpioTableSize ];
  RegisterConfig GPIO_ClockConfigLP[ gpioTableSize ];
  RegisterConfig GPIO_ResetConfig[ gpioTableSize ];
  Configuration::ClockType_t GPIO_SourceClock[ gpioTableSize ];

  const PCC GPIOLookup = {
    GPIO_ClockConfig, GPIO_ClockConfigLP, GPIO_ResetConfig, GPIO_SourceClock, &Thor::LLD::GPIO::InstanceToResourceIndex,
    gpioTableSize
  };

  void GPIOInit()
  {
    using namespace Thor::LLD::GPIO;

    /*------------------------------------------------
    GPIO clock enable register access lookup table
    ------------------------------------------------*/
    GPIO_ClockConfig[ GPIOA_RESOURCE_INDEX ].mask = AHB1ENR_GPIOAEN;
    GPIO_ClockConfig[ GPIOA_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    GPIO_ClockConfig[ GPIOB_RESOURCE_INDEX ].mask = AHB1ENR_GPIOBEN;
    GPIO_ClockConfig[ GPIOB_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    GPIO_ClockConfig[ GPIOC_RESOURCE_INDEX ].mask = AHB1ENR_GPIOCEN;
    GPIO_ClockConfig[ GPIOC_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    GPIO_ClockConfig[ GPIOD_RESOURCE_INDEX ].mask = AHB1ENR_GPIODEN;
    GPIO_ClockConfig[ GPIOD_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    GPIO_ClockConfig[ GPIOE_RESOURCE_INDEX ].mask = AHB1ENR_GPIOEEN;
    GPIO_ClockConfig[ GPIOE_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    GPIO_ClockConfig[ GPIOF_RESOURCE_INDEX ].mask = AHB1ENR_GPIOFEN;
    GPIO_ClockConfig[ GPIOF_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    GPIO_ClockConfig[ GPIOG_RESOURCE_INDEX ].mask = AHB1ENR_GPIOGEN;
    GPIO_ClockConfig[ GPIOG_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    GPIO_ClockConfig[ GPIOH_RESOURCE_INDEX ].mask = AHB1ENR_GPIOHEN;
    GPIO_ClockConfig[ GPIOH_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    /*------------------------------------------------
    GPIO low power clock enable register access lookup table
    ------------------------------------------------*/
    GPIO_ClockConfigLP[ GPIOA_RESOURCE_INDEX ].mask = AHB1LPENR_GPIOALPEN;
    GPIO_ClockConfigLP[ GPIOA_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    GPIO_ClockConfigLP[ GPIOB_RESOURCE_INDEX ].mask = AHB1LPENR_GPIOBLPEN;
    GPIO_ClockConfigLP[ GPIOB_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    GPIO_ClockConfigLP[ GPIOC_RESOURCE_INDEX ].mask = AHB1LPENR_GPIOCLPEN;
    GPIO_ClockConfigLP[ GPIOC_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    GPIO_ClockConfigLP[ GPIOD_RESOURCE_INDEX ].mask = AHB1LPENR_GPIODLPEN;
    GPIO_ClockConfigLP[ GPIOD_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    GPIO_ClockConfigLP[ GPIOE_RESOURCE_INDEX ].mask = AHB1LPENR_GPIOELPEN;
    GPIO_ClockConfigLP[ GPIOE_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    GPIO_ClockConfigLP[ GPIOF_RESOURCE_INDEX ].mask = AHB1LPENR_GPIOFLPEN;
    GPIO_ClockConfigLP[ GPIOF_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    GPIO_ClockConfigLP[ GPIOG_RESOURCE_INDEX ].mask = AHB1LPENR_GPIOGLPEN;
    GPIO_ClockConfigLP[ GPIOG_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    GPIO_ClockConfigLP[ GPIOH_RESOURCE_INDEX ].mask = AHB1LPENR_GPIOHLPEN;
    GPIO_ClockConfigLP[ GPIOH_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    /*------------------------------------------------
    GPIO reset register access lookup table
    ------------------------------------------------*/
    GPIO_ResetConfig[ GPIOA_RESOURCE_INDEX ].mask = AHB1RSTR_GPIOARST;
    GPIO_ResetConfig[ GPIOA_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    GPIO_ResetConfig[ GPIOB_RESOURCE_INDEX ].mask = AHB1RSTR_GPIOBRST;
    GPIO_ResetConfig[ GPIOB_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    GPIO_ResetConfig[ GPIOC_RESOURCE_INDEX ].mask = AHB1RSTR_GPIOCRST;
    GPIO_ResetConfig[ GPIOC_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    GPIO_ResetConfig[ GPIOD_RESOURCE_INDEX ].mask = AHB1RSTR_GPIODRST;
    GPIO_ResetConfig[ GPIOD_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    GPIO_ResetConfig[ GPIOE_RESOURCE_INDEX ].mask = AHB1RSTR_GPIOERST;
    GPIO_ResetConfig[ GPIOE_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    GPIO_ResetConfig[ GPIOF_RESOURCE_INDEX ].mask = AHB1RSTR_GPIOFRST;
    GPIO_ResetConfig[ GPIOF_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    GPIO_ResetConfig[ GPIOG_RESOURCE_INDEX ].mask = AHB1RSTR_GPIOGRST;
    GPIO_ResetConfig[ GPIOG_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    GPIO_ResetConfig[ GPIOH_RESOURCE_INDEX ].mask = AHB1RSTR_GPIOHRST;
    GPIO_ResetConfig[ GPIOH_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    /*------------------------------------------------
    GPIO clocking bus source identifier
    ------------------------------------------------*/
    GPIO_SourceClock[ GPIOA_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    GPIO_SourceClock[ GPIOB_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    GPIO_SourceClock[ GPIOC_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    GPIO_SourceClock[ GPIOD_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    GPIO_SourceClock[ GPIOE_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    GPIO_SourceClock[ GPIOF_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    GPIO_SourceClock[ GPIOG_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    GPIO_SourceClock[ GPIOH_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
  };

}    // namespace Thor::Driver::RCC::LookupTables

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */