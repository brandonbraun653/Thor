/********************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32l432kc.cpp
 *
 *  Description:
 *    GPIO register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_driver.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_mapping.hpp>
#include <Thor/lld/stm32l4x/gpio/variant/hw_gpio_register_stm32l432kc.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_types.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
  /*------------------------------------------------
  Pin Alternate Function Mapping: Port A
  See Table 15 of DS11451
  ------------------------------------------------*/
  /* clang-format off */
  static const AFToReg PA0_AFMap{ 
    { Chimera::GPIO::Alternate::TIM2_CH1,       AF1_TIM2      },
    { Chimera::GPIO::Alternate::USART2_CTS,     AF7_USART2    },
    { Chimera::GPIO::Alternate::COMP1_OUT,      AF12_COMP1    },
    { Chimera::GPIO::Alternate::SAI1_EXTCLK,    AF13_SAI1     },
    { Chimera::GPIO::Alternate::TIM2_ETR,       AF14_TIM2     },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA1_AFMap{ 
    { Chimera::GPIO::Alternate::TIM2_CH2,       AF1_TIM2      },
    { Chimera::GPIO::Alternate::I2C1_SMBA,      AF4_I2C1      },
    { Chimera::GPIO::Alternate::SPI1_SCK,       AF5_SPI1      },
    { Chimera::GPIO::Alternate::USART2_RTS_DE,  AF7_USART2    },
    { Chimera::GPIO::Alternate::TIM15_CH1N,     AF14_TIM15    },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA2_AFMap{ 
    { Chimera::GPIO::Alternate::TIM2_CH3,       AF1_TIM2      },
    { Chimera::GPIO::Alternate::USART2_TX,      AF7_USART2    },
    { Chimera::GPIO::Alternate::LPUART1_TX,     AF8_LPUART1   },
    { Chimera::GPIO::Alternate::QUADSPI_BK1_NCS,AF10_QUADSPI  },
    { Chimera::GPIO::Alternate::COMP2_OUT,      AF12_COMP2    },
    { Chimera::GPIO::Alternate::TIM15_CH1,      AF14_TIM15    },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA3_AFMap{ 
    { Chimera::GPIO::Alternate::TIM2_CH4,       AF1_TIM2      },
    { Chimera::GPIO::Alternate::USART2_RX,      AF7_USART2    },
    { Chimera::GPIO::Alternate::LPUART1_RX,     AF8_LPUART1   },
    { Chimera::GPIO::Alternate::QUADSPI_CLK,    AF10_QUADSPI  },
    { Chimera::GPIO::Alternate::SAI1_MCLK_A,    AF13_SAI1     },
    { Chimera::GPIO::Alternate::TIM15_CH2,      AF14_TIM15    },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA4_AFMap{ 
    { Chimera::GPIO::Alternate::SPI1_NSS,       AF5_SPI1      },
    { Chimera::GPIO::Alternate::SPI3_NSS,       AF6_SPI3      },
    { Chimera::GPIO::Alternate::USART2_CK,      AF7_USART2    },
    { Chimera::GPIO::Alternate::SAI1_FS_B,      AF13_SAI1     },
    { Chimera::GPIO::Alternate::LPTIM2_OUT,     AF14_LPTIM2   },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA5_AFMap{ 
    { Chimera::GPIO::Alternate::TIM2_CH1,       AF1_TIM2      },
    { Chimera::GPIO::Alternate::TIM2_ETR,       AF2_TIM2      },
    { Chimera::GPIO::Alternate::SPI1_SCK,       AF5_SPI1      },
    { Chimera::GPIO::Alternate::LPTIM2_ETR,     AF14_LPTIM2   },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA6_AFMap{ 
    { Chimera::GPIO::Alternate::TIM1_BKIN,      AF1_TIM1      },
    { Chimera::GPIO::Alternate::SPI1_MISO,      AF5_SPI1      },
    { Chimera::GPIO::Alternate::COMP1_OUT,      AF6_COMP1     },
    { Chimera::GPIO::Alternate::USART3_CTS,     AF7_USART3    },
    { Chimera::GPIO::Alternate::LPUART1_CTS,    AF8_LPUART1   },
    { Chimera::GPIO::Alternate::QUADSPI_BK1_IO3,AF10_QUADSPI  },
    { Chimera::GPIO::Alternate::TIM1_BKIN_COMP2,AF12_COMP2    },
    { Chimera::GPIO::Alternate::TIM16_CH1,      AF14_TIM16    },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA7_AFMap{ 
    { Chimera::GPIO::Alternate::TIM1_CH1N,      AF1_TIM1      },
    { Chimera::GPIO::Alternate::I2C3_SCL,       AF4_I2C3      },
    { Chimera::GPIO::Alternate::SPI1_MOSI,      AF5_SPI1      },
    { Chimera::GPIO::Alternate::QUADSPI_BK1_IO2,AF10_QUADSPI  },
    { Chimera::GPIO::Alternate::COMP2_OUT,      AF12_COMP2    },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA8_AFMap{ 
    { Chimera::GPIO::Alternate::MCO,            AF0_MCO       },
    { Chimera::GPIO::Alternate::TIM1_CH1,       AF1_TIM1      },
    { Chimera::GPIO::Alternate::USART1_CK,      AF7_USART1    },
    { Chimera::GPIO::Alternate::SWPMI1_IO,      AF12_SWPMI1   },
    { Chimera::GPIO::Alternate::SAI1_SCK_A,     AF13_SAI1     },
    { Chimera::GPIO::Alternate::LPTIM2_OUT,     AF14_LPTIM2   },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA9_AFMap{ 
    { Chimera::GPIO::Alternate::TIM1_CH2,       AF1_TIM1      },
    { Chimera::GPIO::Alternate::I2C1_SCL,       AF4_I2C1      },
    { Chimera::GPIO::Alternate::USART1_TX,      AF7_USART1    },
    { Chimera::GPIO::Alternate::SAI1_FS_A,      AF13_SAI1     },
    { Chimera::GPIO::Alternate::TIM15_BKIN,     AF14_TIM15    },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA10_AFMap{ 
    { Chimera::GPIO::Alternate::TIM1_CH3,       AF1_TIM1      },
    { Chimera::GPIO::Alternate::I2C1_SDA,       AF4_I2C1      },
    { Chimera::GPIO::Alternate::USART1_RX,      AF7_USART1    },
    { Chimera::GPIO::Alternate::USB_CRS_SYNC,   AF10_USB_FS   },
    { Chimera::GPIO::Alternate::SAI1_SD_A,      AF13_SAI1     },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA11_AFMap{ 
    { Chimera::GPIO::Alternate::TIM1_CH4,       AF1_TIM1      },
    { Chimera::GPIO::Alternate::TIM1_BKIN2,     AF2_TIM1      },
    { Chimera::GPIO::Alternate::SPI1_MISO,      AF5_SPI1      },
    { Chimera::GPIO::Alternate::COMP1_OUT,      AF6_SPI3      },
    { Chimera::GPIO::Alternate::USART1_CTS,     AF7_USART1    },
    { Chimera::GPIO::Alternate::CAN1_RX,        AF9_CAN1      },
    { Chimera::GPIO::Alternate::USB_DM,         AF10_USB_FS   },
    { Chimera::GPIO::Alternate::TIM1_BKIN2_COMP1,AF12_COMP1   },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA12_AFMap{ 
    { Chimera::GPIO::Alternate::TIM1_ETR,       AF1_TIM1      },
    { Chimera::GPIO::Alternate::SPI1_MOSI,      AF5_SPI1      },
    { Chimera::GPIO::Alternate::USART1_RTS_DE,  AF7_USART1    },
    { Chimera::GPIO::Alternate::CAN1_TX,        AF9_CAN1      },
    { Chimera::GPIO::Alternate::USB_DP,         AF10_USB_FS   },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA13_AFMap{ 
    { Chimera::GPIO::Alternate::JTMSSWDIO,      AF0_SWJ       },
    { Chimera::GPIO::Alternate::IR_OUT,         AF1_IR        },
    { Chimera::GPIO::Alternate::USB_NOE,        AF10_USB_FS   },
    { Chimera::GPIO::Alternate::SWPMI1_TX,      AF12_SWPMI1   },
    { Chimera::GPIO::Alternate::SAI1_SD_B,      AF13_SAI1     },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA14_AFMap{ 
    { Chimera::GPIO::Alternate::JTCKSWCLK,      AF0_SWJ       },
    { Chimera::GPIO::Alternate::LPTIM1_OUT,     AF1_LPTIM1    },
    { Chimera::GPIO::Alternate::I2C1_SMBA,      AF4_I2C1      },
    { Chimera::GPIO::Alternate::SWPMI1_RX,      AF12_SWPMI1   },
    { Chimera::GPIO::Alternate::SAI1_FS_B,      AF13_SAI1     },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PA15_AFMap{ 
    { Chimera::GPIO::Alternate::JTDI,           AF0_SWJ       },
    { Chimera::GPIO::Alternate::TIM2_CH1,       AF1_TIM2      },
    { Chimera::GPIO::Alternate::TIM2_ETR,       AF2_TIM2      },
    { Chimera::GPIO::Alternate::USART2_RX,      AF3_USART2    },
    { Chimera::GPIO::Alternate::SPI1_NSS,       AF5_SPI1      },
    { Chimera::GPIO::Alternate::SPI3_NSS,       AF6_SPI3      },
    { Chimera::GPIO::Alternate::USART3_RTS_DE,  AF7_USART3    },
    { Chimera::GPIO::Alternate::TSC_G3_IO1,     AF9_TSC       },
    { Chimera::GPIO::Alternate::SWPMI1_SUSPEND, AF12_SWPMI1   },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const PinToAFMap PA_PinAF{
    { 0,  &PA0_AFMap  },
    { 1,  &PA1_AFMap  },
    { 2,  &PA2_AFMap  },
    { 3,  &PA3_AFMap  },
    { 4,  &PA4_AFMap  },
    { 5,  &PA5_AFMap  },
    { 6,  &PA6_AFMap  },
    { 7,  &PA7_AFMap  },
    { 8,  &PA8_AFMap  },
    { 9,  &PA9_AFMap  },
    { 10, &PA10_AFMap },
    { 11, &PA11_AFMap },
    { 12, &PA12_AFMap },
    { 13, &PA13_AFMap },
    { 14, &PA14_AFMap },
    { 15, &PA15_AFMap }
  };

  /*------------------------------------------------
  Pin Alternate Function Mapping: Port B
  See Table 15 of DS11451
  ------------------------------------------------*/
  static const AFToReg PB0_AFMap{ 
    { Chimera::GPIO::Alternate::TIM1_CH2N,      AF1_TIM2      },
    { Chimera::GPIO::Alternate::SPI1_NSS,       AF5_SPI1      },
    { Chimera::GPIO::Alternate::USART3_CK,      AF7_USART3    },
    { Chimera::GPIO::Alternate::QUADSPI_BK1_IO1,AF10_QUADSPI  },
    { Chimera::GPIO::Alternate::COMP1_OUT,      AF12_COMP1    },
    { Chimera::GPIO::Alternate::SAI1_EXTCLK,    AF13_SAI1     },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PB1_AFMap{ 
    { Chimera::GPIO::Alternate::TIM1_CH3N,      AF1_TIM1      },
    { Chimera::GPIO::Alternate::USART3_RTS_DE,  AF7_USART3    },
    { Chimera::GPIO::Alternate::LPUART1_RTS_DE, AF8_LPUART1   },
    { Chimera::GPIO::Alternate::QUADSPI_BK1_IO0,AF10_QUADSPI  },
    { Chimera::GPIO::Alternate::LPTIM2_IN1,     AF14_LPTIM2   },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PB3_AFMap{ 
    { Chimera::GPIO::Alternate::JTDO_TRACESWO,  AF0_SWJ       },
    { Chimera::GPIO::Alternate::TIM2_CH2,       AF1_TIM2      },
    { Chimera::GPIO::Alternate::SPI1_SCK,       AF5_SPI1      },
    { Chimera::GPIO::Alternate::SPI3_SCK,       AF6_SPI3      },
    { Chimera::GPIO::Alternate::USART1_RTS_DE,  AF7_USART1    },
    { Chimera::GPIO::Alternate::SAI1_SCK_B,     AF13_SAI1     },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PB4_AFMap{ 
    { Chimera::GPIO::Alternate::NJTRST,         AF0_SWJ       },
    { Chimera::GPIO::Alternate::I2C3_SDA,       AF4_I2C3      },
    { Chimera::GPIO::Alternate::SPI1_MISO,      AF5_SPI1      },
    { Chimera::GPIO::Alternate::SPI3_MISO,      AF6_SPI3      },
    { Chimera::GPIO::Alternate::USART1_CTS,     AF7_USART1    },
    { Chimera::GPIO::Alternate::TSC_G2_IO1,     AF9_TSC       },
    { Chimera::GPIO::Alternate::SAI1_MCLK_B,    AF13_SAI1     },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };
  
  static const AFToReg PB5_AFMap{ 
    { Chimera::GPIO::Alternate::LPTIM1_IN1,     AF1_TIM1      },
    { Chimera::GPIO::Alternate::I2C1_SMBA,      AF4_I2C1      },
    { Chimera::GPIO::Alternate::SPI1_MOSI,      AF5_SPI1      },
    { Chimera::GPIO::Alternate::SPI3_MOSI,      AF6_SPI3      },
    { Chimera::GPIO::Alternate::USART1_CK,      AF7_USART1    },
    { Chimera::GPIO::Alternate::TSC_G2_IO2,     AF9_TSC       },
    { Chimera::GPIO::Alternate::COMP2_OUT,      AF12_COMP2    },
    { Chimera::GPIO::Alternate::SAI1_SD_B,      AF13_SAI1     },
    { Chimera::GPIO::Alternate::TIM16_BKIN,     AF14_TIM16    },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PB6_AFMap{ 
    { Chimera::GPIO::Alternate::LPTIM1_ETR,     AF1_LPTIM1    },
    { Chimera::GPIO::Alternate::I2C1_SCL,       AF4_I2C1      },
    { Chimera::GPIO::Alternate::USART1_TX,      AF7_USART1    },
    { Chimera::GPIO::Alternate::TSC_G2_IO3,     AF9_TSC       },
    { Chimera::GPIO::Alternate::SAI1_FS_B,      AF13_SAI1     },
    { Chimera::GPIO::Alternate::TIM16_CH1N,     AF14_TIM16    },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PB7_AFMap{ 
    { Chimera::GPIO::Alternate::LPTIM1_IN2,     AF1_LPTIM1    },
    { Chimera::GPIO::Alternate::I2C1_SDA,       AF4_I2C1      },
    { Chimera::GPIO::Alternate::USART1_RX,      AF7_USART1    },
    { Chimera::GPIO::Alternate::TSC_G2_IO4,     AF9_TSC       },
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const PinToAFMap PB_PinAF{
    { 0, &PB0_AFMap },
    { 1, &PB1_AFMap },
    { 3, &PB3_AFMap },
    { 4, &PB4_AFMap },
    { 5, &PB5_AFMap },
    { 6, &PB6_AFMap },
    { 7, &PB7_AFMap }
  };

  /*------------------------------------------------
  Pin Alternate Function Mapping: Port C
  See Table 15 of DS11451
  ------------------------------------------------*/
  static const AFToReg PC14_AFMap{ 
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const AFToReg PC15_AFMap{ 
    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
  };

  static const PinToAFMap PC_PinAF{
    { 14, &PC14_AFMap },
    { 15, &PC15_AFMap }
  };

  /* clang-format on */

#if defined( EMBEDDED )
  /*-------------------------------------------------
  Memory Mapped Structs to Peripherals
  -------------------------------------------------*/
  RegisterMap *GPIOA_PERIPH = reinterpret_cast<RegisterMap *>( GPIOA_BASE_ADDR );
  RegisterMap *GPIOB_PERIPH = reinterpret_cast<RegisterMap *>( GPIOB_BASE_ADDR );
  RegisterMap *GPIOC_PERIPH = reinterpret_cast<RegisterMap *>( GPIOC_BASE_ADDR );
  RegisterMap *GPIOD_PERIPH = reinterpret_cast<RegisterMap *>( GPIOD_BASE_ADDR );
  RegisterMap *GPIOE_PERIPH = reinterpret_cast<RegisterMap *>( GPIOE_BASE_ADDR );
  RegisterMap *GPIOH_PERIPH = reinterpret_cast<RegisterMap *>( GPIOH_BASE_ADDR );

  /*-------------------------------------------------
  Lookup Tables Defintions
  -------------------------------------------------*/
  /* clang-format off */
  PortMap InstanceToPortMap{ 
    { GPIOA_PERIPH, Chimera::GPIO::Port::PORTA }, 
    { GPIOB_PERIPH, Chimera::GPIO::Port::PORTB },
    { GPIOC_PERIPH, Chimera::GPIO::Port::PORTC }, 
    { GPIOD_PERIPH, Chimera::GPIO::Port::PORTD },
    { GPIOE_PERIPH, Chimera::GPIO::Port::PORTE }, 
    { GPIOH_PERIPH, Chimera::GPIO::Port::PORTH } 
  };

  InstanceMap PortToInstanceMap{ 
    { Chimera::GPIO::Port::PORTA, GPIOA_PERIPH }, 
    { Chimera::GPIO::Port::PORTB, GPIOB_PERIPH },
    { Chimera::GPIO::Port::PORTC, GPIOC_PERIPH }, 
    { Chimera::GPIO::Port::PORTD, GPIOD_PERIPH },
    { Chimera::GPIO::Port::PORTE, GPIOE_PERIPH }, 
    { Chimera::GPIO::Port::PORTH, GPIOH_PERIPH } 
  };

  Thor::LLD::RIndexMap InstanceToResourceIndex{ 
    { reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), GPIOA_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), GPIOB_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), GPIOC_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), GPIOD_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), GPIOE_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), GPIOH_RESOURCE_INDEX } 
  };

  AlternateMap InstanceToAlternateMap{ 
    { GPIOA_PERIPH, &PA_PinAF }, 
    { GPIOB_PERIPH, &PB_PinAF }, 
    { GPIOC_PERIPH, &PC_PinAF }, 
    { GPIOD_PERIPH, nullptr }, 
    { GPIOE_PERIPH, nullptr }, 
    { GPIOH_PERIPH, nullptr } 
  };
  /* clang-format on */

#elif defined( _SIM )
  /*-------------------------------------------------
  Memory Mapped Structs to Virtual Peripherals
  -------------------------------------------------*/
  RegisterMap *GPIOA_PERIPH = nullptr;
  RegisterMap *GPIOB_PERIPH = nullptr;
  RegisterMap *GPIOC_PERIPH = nullptr;
  RegisterMap *GPIOD_PERIPH = nullptr;
  RegisterMap *GPIOE_PERIPH = nullptr;
  RegisterMap *GPIOH_PERIPH = nullptr;

  /*-------------------------------------------------
  Lookup Tables Definitions
  -------------------------------------------------*/
  PortMap InstanceToPortMap;
  IndexMap InstanceToResourceIndex;
  AlternateMap InstanceToAlternateMap;
  InstanceMap PortToInstanceMap;
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
    GPIOH_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ GPIOA_RESOURCE_INDEX ] = GPIOA_PERIPH;
    PeripheralList[ GPIOB_RESOURCE_INDEX ] = GPIOB_PERIPH;
    PeripheralList[ GPIOC_RESOURCE_INDEX ] = GPIOC_PERIPH;
    PeripheralList[ GPIOD_RESOURCE_INDEX ] = GPIOD_PERIPH;
    PeripheralList[ GPIOE_RESOURCE_INDEX ] = GPIOE_PERIPH;
    PeripheralList[ GPIOH_RESOURCE_INDEX ] = GPIOH_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), Chimera::GPIO::Port::PORTA );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), Chimera::GPIO::Port::PORTB );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), Chimera::GPIO::Port::PORTC );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), Chimera::GPIO::Port::PORTD );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), Chimera::GPIO::Port::PORTE );
    InstanceToPortMap.append( reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), Chimera::GPIO::Port::PORTH );


    PortToInstanceMap.append( Chimera::GPIO::Port::PORTA, GPIOA_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTB, GPIOB_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTC, GPIOC_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTD, GPIOD_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTE, GPIOE_PERIPH );
    PortToInstanceMap.append( Chimera::GPIO::Port::PORTH, GPIOH_PERIPH );

    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), GPIOA_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), GPIOB_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), GPIOC_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), GPIOD_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), GPIOE_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), GPIOH_RESOURCE_INDEX );

    InstanceToAlternateMap.append( reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), nullptr );
    InstanceToAlternateMap.append( reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), nullptr );
    InstanceToAlternateMap.append( reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), nullptr );
    InstanceToAlternateMap.append( reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), nullptr );
    InstanceToAlternateMap.append( reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), nullptr );
    InstanceToAlternateMap.append( reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), nullptr );
#endif

    /*------------------------------------------------
    Register the Alternate Configurations
    ------------------------------------------------*/
//    InstanceToAlternateMap.assign( GPIOA_PERIPH, &PA_PinAF );
//    InstanceToAlternateMap.assign( GPIOB_PERIPH, &PB_PinAF );
//    InstanceToAlternateMap.assign( GPIOC_PERIPH, &PC_PinAF );
//    InstanceToAlternateMap.assign( GPIOD_PERIPH, nullptr );
//    InstanceToAlternateMap.assign( GPIOE_PERIPH, nullptr );
//    InstanceToAlternateMap.assign( GPIOH_PERIPH, nullptr );
  }
}    // namespace Thor::LLD::GPIO

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_gpio_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig GPIO_ClockConfig[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
  RegisterConfig GPIO_ResetConfig[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
  Chimera::Clock::Bus GPIO_SourceClock[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];

  PCC GPIOLookup = { GPIO_ClockConfig,
                     nullptr,
                     GPIO_ResetConfig,
                     GPIO_SourceClock,
                     Thor::LLD::GPIO::NUM_GPIO_PERIPHS,
                     Thor::LLD::GPIO::getResourceIndex };

  void GPIOInit()
  {
    using namespace Thor::LLD::GPIO;

    /*------------------------------------------------
    GPIO clock enable register access lookup table
    ------------------------------------------------*/
    GPIO_ClockConfig[ GPIOA_RESOURCE_INDEX ].mask = AHB2ENR_GPIOAEN;
    GPIO_ClockConfig[ GPIOA_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2ENR;

    GPIO_ClockConfig[ GPIOB_RESOURCE_INDEX ].mask = AHB2ENR_GPIOBEN;
    GPIO_ClockConfig[ GPIOB_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2ENR;

    GPIO_ClockConfig[ GPIOC_RESOURCE_INDEX ].mask = AHB2ENR_GPIOCEN;
    GPIO_ClockConfig[ GPIOC_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2ENR;

    GPIO_ClockConfig[ GPIOH_RESOURCE_INDEX ].mask = AHB2ENR_GPIOHEN;
    GPIO_ClockConfig[ GPIOH_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2ENR;

    /*------------------------------------------------
    GPIO reset register access lookup table
    ------------------------------------------------*/
    GPIO_ResetConfig[ GPIOA_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOARST;
    GPIO_ResetConfig[ GPIOA_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    GPIO_ResetConfig[ GPIOB_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOBRST;
    GPIO_ResetConfig[ GPIOB_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    GPIO_ResetConfig[ GPIOC_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOCRST;
    GPIO_ResetConfig[ GPIOC_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    GPIO_ResetConfig[ GPIOH_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOHRST;
    GPIO_ResetConfig[ GPIOH_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    /*------------------------------------------------
    GPIO clocking bus source identifier
    ------------------------------------------------*/
    GPIO_SourceClock[ GPIOA_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
    GPIO_SourceClock[ GPIOB_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
    GPIO_SourceClock[ GPIOC_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
    GPIO_SourceClock[ GPIOH_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_GPIO */
