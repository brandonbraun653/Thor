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
//  static const AFToReg PA0_AFMap{
//    { Chimera::GPIO::Alternate::TIM2_CH1,       AF1_TIM2      },
//    { Chimera::GPIO::Alternate::USART2_CTS,     AF7_USART2    },
//    { Chimera::GPIO::Alternate::COMP1_OUT,      AF12_COMP1    },
//    { Chimera::GPIO::Alternate::SAI1_EXTCLK,    AF13_SAI1     },
//    { Chimera::GPIO::Alternate::TIM2_ETR,       AF14_TIM2     },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA1_AFMap{
//    { Chimera::GPIO::Alternate::TIM2_CH2,       AF1_TIM2      },
//    { Chimera::GPIO::Alternate::I2C1_SMBA,      AF4_I2C1      },
//    { Chimera::GPIO::Alternate::SPI1_SCK,       AF5_SPI1      },
//    { Chimera::GPIO::Alternate::USART2_RTS_DE,  AF7_USART2    },
//    { Chimera::GPIO::Alternate::TIM15_CH1N,     AF14_TIM15    },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA2_AFMap{
//    { Chimera::GPIO::Alternate::TIM2_CH3,       AF1_TIM2      },
//    { Chimera::GPIO::Alternate::USART2_TX,      AF7_USART2    },
//    { Chimera::GPIO::Alternate::LPUART1_TX,     AF8_LPUART1   },
//    { Chimera::GPIO::Alternate::QUADSPI_BK1_NCS,AF10_QUADSPI  },
//    { Chimera::GPIO::Alternate::COMP2_OUT,      AF12_COMP2    },
//    { Chimera::GPIO::Alternate::TIM15_CH1,      AF14_TIM15    },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA3_AFMap{
//    { Chimera::GPIO::Alternate::TIM2_CH4,       AF1_TIM2      },
//    { Chimera::GPIO::Alternate::USART2_RX,      AF7_USART2    },
//    { Chimera::GPIO::Alternate::LPUART1_RX,     AF8_LPUART1   },
//    { Chimera::GPIO::Alternate::QUADSPI_CLK,    AF10_QUADSPI  },
//    { Chimera::GPIO::Alternate::SAI1_MCLK_A,    AF13_SAI1     },
//    { Chimera::GPIO::Alternate::TIM15_CH2,      AF14_TIM15    },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA4_AFMap{
//    { Chimera::GPIO::Alternate::SPI1_NSS,       AF5_SPI1      },
//    { Chimera::GPIO::Alternate::SPI3_NSS,       AF6_SPI3      },
//    { Chimera::GPIO::Alternate::USART2_CK,      AF7_USART2    },
//    { Chimera::GPIO::Alternate::SAI1_FS_B,      AF13_SAI1     },
//    { Chimera::GPIO::Alternate::LPTIM2_OUT,     AF14_LPTIM2   },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA5_AFMap{
//    { Chimera::GPIO::Alternate::TIM2_CH1,       AF1_TIM2      },
//    { Chimera::GPIO::Alternate::TIM2_ETR,       AF2_TIM2      },
//    { Chimera::GPIO::Alternate::SPI1_SCK,       AF5_SPI1      },
//    { Chimera::GPIO::Alternate::LPTIM2_ETR,     AF14_LPTIM2   },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA6_AFMap{
//    { Chimera::GPIO::Alternate::TIM1_BKIN,      AF1_TIM1      },
//    { Chimera::GPIO::Alternate::SPI1_MISO,      AF5_SPI1      },
//    { Chimera::GPIO::Alternate::COMP1_OUT,      AF6_COMP1     },
//    { Chimera::GPIO::Alternate::USART3_CTS,     AF7_USART3    },
//    { Chimera::GPIO::Alternate::LPUART1_CTS,    AF8_LPUART1   },
//    { Chimera::GPIO::Alternate::QUADSPI_BK1_IO3,AF10_QUADSPI  },
//    { Chimera::GPIO::Alternate::TIM1_BKIN_COMP2,AF12_COMP2    },
//    { Chimera::GPIO::Alternate::TIM16_CH1,      AF14_TIM16    },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA7_AFMap{
//    { Chimera::GPIO::Alternate::TIM1_CH1N,      AF1_TIM1      },
//    { Chimera::GPIO::Alternate::I2C3_SCL,       AF4_I2C3      },
//    { Chimera::GPIO::Alternate::SPI1_MOSI,      AF5_SPI1      },
//    { Chimera::GPIO::Alternate::QUADSPI_BK1_IO2,AF10_QUADSPI  },
//    { Chimera::GPIO::Alternate::COMP2_OUT,      AF12_COMP2    },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA8_AFMap{
//    { Chimera::GPIO::Alternate::MCO,            AF0_MCO       },
//    { Chimera::GPIO::Alternate::TIM1_CH1,       AF1_TIM1      },
//    { Chimera::GPIO::Alternate::USART1_CK,      AF7_USART1    },
//    { Chimera::GPIO::Alternate::SWPMI1_IO,      AF12_SWPMI1   },
//    { Chimera::GPIO::Alternate::SAI1_SCK_A,     AF13_SAI1     },
//    { Chimera::GPIO::Alternate::LPTIM2_OUT,     AF14_LPTIM2   },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA9_AFMap{
//    { Chimera::GPIO::Alternate::TIM1_CH2,       AF1_TIM1      },
//    { Chimera::GPIO::Alternate::I2C1_SCL,       AF4_I2C1      },
//    { Chimera::GPIO::Alternate::USART1_TX,      AF7_USART1    },
//    { Chimera::GPIO::Alternate::SAI1_FS_A,      AF13_SAI1     },
//    { Chimera::GPIO::Alternate::TIM15_BKIN,     AF14_TIM15    },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA10_AFMap{
//    { Chimera::GPIO::Alternate::TIM1_CH3,       AF1_TIM1      },
//    { Chimera::GPIO::Alternate::I2C1_SDA,       AF4_I2C1      },
//    { Chimera::GPIO::Alternate::USART1_RX,      AF7_USART1    },
//    { Chimera::GPIO::Alternate::USB_CRS_SYNC,   AF10_USB_FS   },
//    { Chimera::GPIO::Alternate::SAI1_SD_A,      AF13_SAI1     },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA11_AFMap{
//    { Chimera::GPIO::Alternate::TIM1_CH4,       AF1_TIM1      },
//    { Chimera::GPIO::Alternate::TIM1_BKIN2,     AF2_TIM1      },
//    { Chimera::GPIO::Alternate::SPI1_MISO,      AF5_SPI1      },
//    { Chimera::GPIO::Alternate::COMP1_OUT,      AF6_SPI3      },
//    { Chimera::GPIO::Alternate::USART1_CTS,     AF7_USART1    },
//    { Chimera::GPIO::Alternate::CAN1_RX,        AF9_CAN1      },
//    { Chimera::GPIO::Alternate::USB_DM,         AF10_USB_FS   },
//    { Chimera::GPIO::Alternate::TIM1_BKIN2_COMP1,AF12_COMP1   },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA12_AFMap{
//    { Chimera::GPIO::Alternate::TIM1_ETR,       AF1_TIM1      },
//    { Chimera::GPIO::Alternate::SPI1_MOSI,      AF5_SPI1      },
//    { Chimera::GPIO::Alternate::USART1_RTS_DE,  AF7_USART1    },
//    { Chimera::GPIO::Alternate::CAN1_TX,        AF9_CAN1      },
//    { Chimera::GPIO::Alternate::USB_DP,         AF10_USB_FS   },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA13_AFMap{
//    { Chimera::GPIO::Alternate::JTMSSWDIO,      AF0_SWJ       },
//    { Chimera::GPIO::Alternate::IR_OUT,         AF1_IR        },
//    { Chimera::GPIO::Alternate::USB_NOE,        AF10_USB_FS   },
//    { Chimera::GPIO::Alternate::SWPMI1_TX,      AF12_SWPMI1   },
//    { Chimera::GPIO::Alternate::SAI1_SD_B,      AF13_SAI1     },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA14_AFMap{
//    { Chimera::GPIO::Alternate::JTCKSWCLK,      AF0_SWJ       },
//    { Chimera::GPIO::Alternate::LPTIM1_OUT,     AF1_LPTIM1    },
//    { Chimera::GPIO::Alternate::I2C1_SMBA,      AF4_I2C1      },
//    { Chimera::GPIO::Alternate::SWPMI1_RX,      AF12_SWPMI1   },
//    { Chimera::GPIO::Alternate::SAI1_FS_B,      AF13_SAI1     },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PA15_AFMap{
//    { Chimera::GPIO::Alternate::JTDI,           AF0_SWJ       },
//    { Chimera::GPIO::Alternate::TIM2_CH1,       AF1_TIM2      },
//    { Chimera::GPIO::Alternate::TIM2_ETR,       AF2_TIM2      },
//    { Chimera::GPIO::Alternate::USART2_RX,      AF3_USART2    },
//    { Chimera::GPIO::Alternate::SPI1_NSS,       AF5_SPI1      },
//    { Chimera::GPIO::Alternate::SPI3_NSS,       AF6_SPI3      },
//    { Chimera::GPIO::Alternate::USART3_RTS_DE,  AF7_USART3    },
//    { Chimera::GPIO::Alternate::TSC_G3_IO1,     AF9_TSC       },
//    { Chimera::GPIO::Alternate::SWPMI1_SUSPEND, AF12_SWPMI1   },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const PinToAFMap PA_PinAF{
//    { 0,  &PA0_AFMap  },
//    { 1,  &PA1_AFMap  },
//    { 2,  &PA2_AFMap  },
//    { 3,  &PA3_AFMap  },
//    { 4,  &PA4_AFMap  },
//    { 5,  &PA5_AFMap  },
//    { 6,  &PA6_AFMap  },
//    { 7,  &PA7_AFMap  },
//    { 8,  &PA8_AFMap  },
//    { 9,  &PA9_AFMap  },
//    { 10, &PA10_AFMap },
//    { 11, &PA11_AFMap },
//    { 12, &PA12_AFMap },
//    { 13, &PA13_AFMap },
//    { 14, &PA14_AFMap },
//    { 15, &PA15_AFMap }
//  };

  /*------------------------------------------------
  Pin Alternate Function Mapping: Port B
  See Table 15 of DS11451
  ------------------------------------------------*/
//  static const AFToReg PB0_AFMap{
//    { Chimera::GPIO::Alternate::TIM1_CH2N,      AF1_TIM2      },
//    { Chimera::GPIO::Alternate::SPI1_NSS,       AF5_SPI1      },
//    { Chimera::GPIO::Alternate::USART3_CK,      AF7_USART3    },
//    { Chimera::GPIO::Alternate::QUADSPI_BK1_IO1,AF10_QUADSPI  },
//    { Chimera::GPIO::Alternate::COMP1_OUT,      AF12_COMP1    },
//    { Chimera::GPIO::Alternate::SAI1_EXTCLK,    AF13_SAI1     },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PB1_AFMap{
//    { Chimera::GPIO::Alternate::TIM1_CH3N,      AF1_TIM1      },
//    { Chimera::GPIO::Alternate::USART3_RTS_DE,  AF7_USART3    },
//    { Chimera::GPIO::Alternate::LPUART1_RTS_DE, AF8_LPUART1   },
//    { Chimera::GPIO::Alternate::QUADSPI_BK1_IO0,AF10_QUADSPI  },
//    { Chimera::GPIO::Alternate::LPTIM2_IN1,     AF14_LPTIM2   },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PB3_AFMap{
//    { Chimera::GPIO::Alternate::JTDO_TRACESWO,  AF0_SWJ       },
//    { Chimera::GPIO::Alternate::TIM2_CH2,       AF1_TIM2      },
//    { Chimera::GPIO::Alternate::SPI1_SCK,       AF5_SPI1      },
//    { Chimera::GPIO::Alternate::SPI3_SCK,       AF6_SPI3      },
//    { Chimera::GPIO::Alternate::USART1_RTS_DE,  AF7_USART1    },
//    { Chimera::GPIO::Alternate::SAI1_SCK_B,     AF13_SAI1     },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PB4_AFMap{
//    { Chimera::GPIO::Alternate::NJTRST,         AF0_SWJ       },
//    { Chimera::GPIO::Alternate::I2C3_SDA,       AF4_I2C3      },
//    { Chimera::GPIO::Alternate::SPI1_MISO,      AF5_SPI1      },
//    { Chimera::GPIO::Alternate::SPI3_MISO,      AF6_SPI3      },
//    { Chimera::GPIO::Alternate::USART1_CTS,     AF7_USART1    },
//    { Chimera::GPIO::Alternate::TSC_G2_IO1,     AF9_TSC       },
//    { Chimera::GPIO::Alternate::SAI1_MCLK_B,    AF13_SAI1     },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PB5_AFMap{
//    { Chimera::GPIO::Alternate::LPTIM1_IN1,     AF1_TIM1      },
//    { Chimera::GPIO::Alternate::I2C1_SMBA,      AF4_I2C1      },
//    { Chimera::GPIO::Alternate::SPI1_MOSI,      AF5_SPI1      },
//    { Chimera::GPIO::Alternate::SPI3_MOSI,      AF6_SPI3      },
//    { Chimera::GPIO::Alternate::USART1_CK,      AF7_USART1    },
//    { Chimera::GPIO::Alternate::TSC_G2_IO2,     AF9_TSC       },
//    { Chimera::GPIO::Alternate::COMP2_OUT,      AF12_COMP2    },
//    { Chimera::GPIO::Alternate::SAI1_SD_B,      AF13_SAI1     },
//    { Chimera::GPIO::Alternate::TIM16_BKIN,     AF14_TIM16    },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PB6_AFMap{
//    { Chimera::GPIO::Alternate::LPTIM1_ETR,     AF1_LPTIM1    },
//    { Chimera::GPIO::Alternate::I2C1_SCL,       AF4_I2C1      },
//    { Chimera::GPIO::Alternate::USART1_TX,      AF7_USART1    },
//    { Chimera::GPIO::Alternate::TSC_G2_IO3,     AF9_TSC       },
//    { Chimera::GPIO::Alternate::SAI1_FS_B,      AF13_SAI1     },
//    { Chimera::GPIO::Alternate::TIM16_CH1N,     AF14_TIM16    },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PB7_AFMap{
//    { Chimera::GPIO::Alternate::LPTIM1_IN2,     AF1_LPTIM1    },
//    { Chimera::GPIO::Alternate::I2C1_SDA,       AF4_I2C1      },
//    { Chimera::GPIO::Alternate::USART1_RX,      AF7_USART1    },
//    { Chimera::GPIO::Alternate::TSC_G2_IO4,     AF9_TSC       },
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const PinToAFMap PB_PinAF{
//    { 0, &PB0_AFMap },
//    { 1, &PB1_AFMap },
//    { 3, &PB3_AFMap },
//    { 4, &PB4_AFMap },
//    { 5, &PB5_AFMap },
//    { 6, &PB6_AFMap },
//    { 7, &PB7_AFMap }
//  };

  /*------------------------------------------------
  Pin Alternate Function Mapping: Port C
  See Table 15 of DS11451
  ------------------------------------------------*/
//  static const AFToReg PC14_AFMap{
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const AFToReg PC15_AFMap{
//    { Chimera::GPIO::Alternate::EVENTOUT,       AF15_EVENTOUT }
//  };
//
//  static const PinToAFMap PC_PinAF{
//    { 14, &PC14_AFMap },
//    { 15, &PC15_AFMap }
//  };

  /* clang-format on */


  namespace Internal
  { /* clang-format off */
    /*-------------------------------------------------------------------------------
    PORT A CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_a0_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a1_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a2_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a3_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a4_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a5_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a6_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a7_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a8_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a9_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a10_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a11_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a12_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a13_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a14_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    
    static const AlternateFunc port_a15_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT },
      // More here
    };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/

    // TODO: Conver list size to sizeof

    static const PinAttributes port_a_pin_attributes[ GPIOA_NUM_PINS ] = {
      { .pinID = 0, .afListSize = 1, .altFunc = port_a0_alt_func },
      { .pinID = 1, .afListSize = 1, .altFunc = port_a1_alt_func },      
      { .pinID = 2, .afListSize = 1, .altFunc = port_a2_alt_func },
      { .pinID = 3, .afListSize = 1, .altFunc = port_a3_alt_func },
      { .pinID = 4, .afListSize = 1, .altFunc = port_a4_alt_func },
      { .pinID = 5, .afListSize = 1, .altFunc = port_a5_alt_func },
      { .pinID = 6, .afListSize = 1, .altFunc = port_a6_alt_func },
      { .pinID = 7, .afListSize = 1, .altFunc = port_a7_alt_func },
      { .pinID = 8, .afListSize = 1, .altFunc = port_a8_alt_func },
      { .pinID = 9, .afListSize = 1, .altFunc = port_a9_alt_func },
      { .pinID = 10, .afListSize = 1, .altFunc = port_a10_alt_func },
      { .pinID = 11, .afListSize = 1, .altFunc = port_a11_alt_func },
      { .pinID = 12, .afListSize = 1, .altFunc = port_a12_alt_func },
      { .pinID = 13, .afListSize = 1, .altFunc = port_a13_alt_func },
      { .pinID = 14, .afListSize = 1, .altFunc = port_a14_alt_func },
      { .pinID = 15, .afListSize = 1, .altFunc = port_a15_alt_func }
    };

    
    /*-------------------------------------------------------------------------------
    PORT B CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_b0_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT }
    };

    static const AlternateFunc port_b1_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT }
    };

    static const AlternateFunc port_b3_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT }
    };

    static const AlternateFunc port_b4_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT }
    };

    static const AlternateFunc port_b5_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT }
    };

    static const AlternateFunc port_b6_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT }
    };

    static const AlternateFunc port_b7_alt_func[ 1 ] = { 
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT }
    };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_b_pin_attributes[ GPIOB_NUM_PINS ] = {
      { .pinID = 0, .afListSize = 1, .altFunc = port_b0_alt_func },
      { .pinID = 1, .afListSize = 1, .altFunc = port_b1_alt_func },
      // Pin 2 not supported
      { .pinID = 3, .afListSize = 1, .altFunc = port_b3_alt_func },
      { .pinID = 4, .afListSize = 1, .altFunc = port_b4_alt_func },
      { .pinID = 5, .afListSize = 1, .altFunc = port_b5_alt_func },
      { .pinID = 6, .afListSize = 1, .altFunc = port_b6_alt_func },
      { .pinID = 7, .afListSize = 1, .altFunc = port_b7_alt_func },
    };


    /*-------------------------------------------------------------------------------
    PORT C CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_c14_alt_func[ 1 ] = { {
        .registerAltFunc = AF15_EVENTOUT,
        .chimeraAltFunc  = Chimera::GPIO::Alternate::EVENTOUT,
    } };

    static const AlternateFunc port_c15_alt_func[ 1 ] = { {
        .registerAltFunc = AF15_EVENTOUT,
        .chimeraAltFunc  = Chimera::GPIO::Alternate::EVENTOUT,
    } };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_c_pin_attributes[ GPIOC_NUM_PINS ] = {
      { .pinID = 14, .afListSize = 1, .altFunc = port_c14_alt_func },
      { .pinID = 15, .afListSize = 1, .altFunc = port_c15_alt_func }
    };
    /* clang-format on */
  }    // namespace Internal

  extern const PortAttributes portAttributes[ NUM_GPIO_PERIPHS ] = {
    // GPIO A
    { 
      .portID      = Chimera::GPIO::Port::PORTA,
      .pinListSize = GPIOA_NUM_PINS,
      .pins        = Internal::port_a_pin_attributes
    },

    // GPIO B
    { 
      .portID      = Chimera::GPIO::Port::PORTB, 
      .pinListSize = GPIOB_NUM_PINS,
      .pins        = Internal::port_b_pin_attributes
    },

    // GPIO C
    { 
      .portID      = Chimera::GPIO::Port::PORTC, 
      .pinListSize = GPIOC_NUM_PINS, 
      .pins        = Internal::port_c_pin_attributes 
    }
  };


  void initializeRegisters()
  {
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

    /*------------------------------------------------
    GPIO reset register access lookup table
    ------------------------------------------------*/
    GPIO_ResetConfig[ GPIOA_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOARST;
    GPIO_ResetConfig[ GPIOA_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    GPIO_ResetConfig[ GPIOB_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOBRST;
    GPIO_ResetConfig[ GPIOB_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    GPIO_ResetConfig[ GPIOC_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOCRST;
    GPIO_ResetConfig[ GPIOC_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    /*------------------------------------------------
    GPIO clocking bus source identifier
    ------------------------------------------------*/
    GPIO_SourceClock[ GPIOA_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
    GPIO_SourceClock[ GPIOB_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
    GPIO_SourceClock[ GPIOC_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_GPIO */
