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
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_prv_data.hpp>
#include <Thor/lld/interface/gpio/sim/gpio_sim_variant.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Pin Alternate Configuration Mapping: See Table 15 of DS11451
  -------------------------------------------------------------------------------*/
  namespace Internal
  { /* clang-format off */
    /*-------------------------------------------------------------------------------
    PORT A CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_a0_alt_func[] = {
      { .registerAltFunc = AF1_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_CH1    },
      { .registerAltFunc = AF7_USART2,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART2_CTS  },
      { .registerAltFunc = AF12_COMP1,        .chimeraAltFunc = Chimera::GPIO::Alternate::COMP1_OUT   },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_EXTCLK },
      { .registerAltFunc = AF14_TIM2,         .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_ETR    },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_a1_alt_func[] = {
      { .registerAltFunc = AF1_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_CH2      },
      { .registerAltFunc = AF4_I2C1,          .chimeraAltFunc = Chimera::GPIO::Alternate::I2C1_SMBA     },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_SCK      },
      { .registerAltFunc = AF7_USART2,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART2_RTS_DE },
      { .registerAltFunc = AF14_TIM15,        .chimeraAltFunc = Chimera::GPIO::Alternate::TIM15_CH1N    },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT      },
    };

    static const AlternateFunc port_a2_alt_func[] = {
      { .registerAltFunc = AF1_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_CH3        },
      { .registerAltFunc = AF7_USART2,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART2_TX       },
      { .registerAltFunc = AF8_LPUART1,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPUART1_TX      },
      { .registerAltFunc = AF10_QUADSPI,      .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_BK1_NCS },
      { .registerAltFunc = AF12_COMP2,        .chimeraAltFunc = Chimera::GPIO::Alternate::COMP2_OUT       },
      { .registerAltFunc = AF14_TIM15,        .chimeraAltFunc = Chimera::GPIO::Alternate::TIM15_CH1       },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT        },
    };

    static const AlternateFunc port_a3_alt_func[] = {
      { .registerAltFunc = AF1_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_CH4    },
      { .registerAltFunc = AF7_USART2,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART2_RX   },
      { .registerAltFunc = AF8_LPUART1,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPUART1_RX  },
      { .registerAltFunc = AF10_QUADSPI,      .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_CLK },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_MCLK_A },
      { .registerAltFunc = AF14_TIM15,        .chimeraAltFunc = Chimera::GPIO::Alternate::TIM15_CH2   },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_a4_alt_func[] = {
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_NSS    },
      { .registerAltFunc = AF6_SPI3,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI3_NSS    },
      { .registerAltFunc = AF7_USART2,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART2_CK   },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_FS_B   },
      { .registerAltFunc = AF14_LPTIM2,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPTIM2_OUT  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_a5_alt_func[] = {
      { .registerAltFunc = AF1_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_CH1    },
      { .registerAltFunc = AF2_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_ETR    },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_SCK    },
      { .registerAltFunc = AF14_LPTIM2,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPTIM2_ETR  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_a6_alt_func[] = {
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_BKIN       },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_MISO       },
      { .registerAltFunc = AF6_COMP1,         .chimeraAltFunc = Chimera::GPIO::Alternate::COMP1_OUT       },
      { .registerAltFunc = AF7_USART3,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART3_CTS      },
      { .registerAltFunc = AF8_LPUART1,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPUART1_CTS     },
      { .registerAltFunc = AF10_QUADSPI,      .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_BK1_IO3 },
      { .registerAltFunc = AF12_COMP2,        .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_BKIN_COMP2 },
      { .registerAltFunc = AF14_TIM16,        .chimeraAltFunc = Chimera::GPIO::Alternate::TIM16_CH1       },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT        },
    };

    static const AlternateFunc port_a7_alt_func[] = {
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_CH1N       },
      { .registerAltFunc = AF4_I2C3,          .chimeraAltFunc = Chimera::GPIO::Alternate::I2C3_SCL        },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_MOSI       },
      { .registerAltFunc = AF10_QUADSPI,      .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_BK1_IO2 },
      { .registerAltFunc = AF12_COMP2,        .chimeraAltFunc = Chimera::GPIO::Alternate::COMP2_OUT       },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT        },
    };

    static const AlternateFunc port_a8_alt_func[] = {
      { .registerAltFunc = AF0_MCO,           .chimeraAltFunc = Chimera::GPIO::Alternate::MCO         },
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_CH1    },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_CK   },
      { .registerAltFunc = AF12_SWPMI1,       .chimeraAltFunc = Chimera::GPIO::Alternate::SWPMI1_IO   },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_SCK_A  },
      { .registerAltFunc = AF14_LPTIM2,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPTIM2_OUT  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_a9_alt_func[] = {
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_CH2    },
      { .registerAltFunc = AF4_I2C1,          .chimeraAltFunc = Chimera::GPIO::Alternate::I2C1_SCL    },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_TX   },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_FS_A   },
      { .registerAltFunc = AF14_TIM15,        .chimeraAltFunc = Chimera::GPIO::Alternate::TIM15_BKIN  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_a10_alt_func[] = {
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_CH3      },
      { .registerAltFunc = AF4_I2C1,          .chimeraAltFunc = Chimera::GPIO::Alternate::I2C1_SDA      },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_RX     },
      { .registerAltFunc = AF10_USB_FS,       .chimeraAltFunc = Chimera::GPIO::Alternate::USB_CRS_SYNC  },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_SD_A     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT      },
    };

    static const AlternateFunc port_a11_alt_func[] = {
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_CH4          },
      { .registerAltFunc = AF2_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_BKIN2        },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_MISO         },
      { .registerAltFunc = AF6_SPI3,          .chimeraAltFunc = Chimera::GPIO::Alternate::COMP1_OUT         },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_CTS        },
      { .registerAltFunc = AF9_CAN1,          .chimeraAltFunc = Chimera::GPIO::Alternate::CAN1_RX           },
      { .registerAltFunc = AF10_USB_FS,       .chimeraAltFunc = Chimera::GPIO::Alternate::USB_DM            },
      { .registerAltFunc = AF12_COMP1,        .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_BKIN2_COMP1  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT          },
    };

    static const AlternateFunc port_a12_alt_func[] = {
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_ETR      },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_MOSI     },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_RTS_DE },
      { .registerAltFunc = AF9_CAN1,          .chimeraAltFunc = Chimera::GPIO::Alternate::CAN1_TX       },
      { .registerAltFunc = AF10_USB_FS,       .chimeraAltFunc = Chimera::GPIO::Alternate::USB_DP        },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT      },
    };

    static const AlternateFunc port_a13_alt_func[] = {
      { .registerAltFunc = AF0_SWJ,           .chimeraAltFunc = Chimera::GPIO::Alternate::JTMSSWDIO },
      { .registerAltFunc = AF1_IR,            .chimeraAltFunc = Chimera::GPIO::Alternate::IR_OUT    },
      { .registerAltFunc = AF10_USB_FS,       .chimeraAltFunc = Chimera::GPIO::Alternate::USB_NOE   },
      { .registerAltFunc = AF12_SWPMI1,       .chimeraAltFunc = Chimera::GPIO::Alternate::SWPMI1_TX },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_SD_B },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT  },
    };

    static const AlternateFunc port_a14_alt_func[] = {
      { .registerAltFunc = AF0_SWJ,           .chimeraAltFunc = Chimera::GPIO::Alternate::JTCKSWCLK   },
      { .registerAltFunc = AF1_LPTIM1,        .chimeraAltFunc = Chimera::GPIO::Alternate::LPTIM1_OUT  },
      { .registerAltFunc = AF4_I2C1,          .chimeraAltFunc = Chimera::GPIO::Alternate::I2C1_SMBA   },
      { .registerAltFunc = AF12_SWPMI1,       .chimeraAltFunc = Chimera::GPIO::Alternate::SWPMI1_RX   },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_FS_B   },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_a15_alt_func[] = {
      { .registerAltFunc = AF0_SWJ,           .chimeraAltFunc = Chimera::GPIO::Alternate::JTDI            },
      { .registerAltFunc = AF1_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_CH1        },
      { .registerAltFunc = AF2_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_ETR        },
      { .registerAltFunc = AF3_USART2,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART2_RX       },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_NSS        },
      { .registerAltFunc = AF6_SPI3,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI3_NSS        },
      { .registerAltFunc = AF7_USART3,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART3_RTS_DE   },
      { .registerAltFunc = AF9_TSC,           .chimeraAltFunc = Chimera::GPIO::Alternate::TSC_G3_IO1      },
      { .registerAltFunc = AF12_SWPMI1,       .chimeraAltFunc = Chimera::GPIO::Alternate::SWPMI1_SUSPEND  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT        },
    };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_a_pin_attributes[ GPIOA_NUM_PINS ] = {
      { .pinID = 0,  .afListSize = ARRAY_COUNT( port_a0_alt_func ),  .altFunc = port_a0_alt_func  },
      { .pinID = 1,  .afListSize = ARRAY_COUNT( port_a1_alt_func ),  .altFunc = port_a1_alt_func  },
      { .pinID = 2,  .afListSize = ARRAY_COUNT( port_a2_alt_func ),  .altFunc = port_a2_alt_func  },
      { .pinID = 3,  .afListSize = ARRAY_COUNT( port_a3_alt_func ),  .altFunc = port_a3_alt_func  },
      { .pinID = 4,  .afListSize = ARRAY_COUNT( port_a4_alt_func ),  .altFunc = port_a4_alt_func  },
      { .pinID = 5,  .afListSize = ARRAY_COUNT( port_a5_alt_func ),  .altFunc = port_a5_alt_func  },
      { .pinID = 6,  .afListSize = ARRAY_COUNT( port_a6_alt_func ),  .altFunc = port_a6_alt_func  },
      { .pinID = 7,  .afListSize = ARRAY_COUNT( port_a7_alt_func ),  .altFunc = port_a7_alt_func  },
      { .pinID = 8,  .afListSize = ARRAY_COUNT( port_a8_alt_func ),  .altFunc = port_a8_alt_func  },
      { .pinID = 9,  .afListSize = ARRAY_COUNT( port_a9_alt_func ),  .altFunc = port_a9_alt_func  },
      { .pinID = 10, .afListSize = ARRAY_COUNT( port_a10_alt_func ), .altFunc = port_a10_alt_func },
      { .pinID = 11, .afListSize = ARRAY_COUNT( port_a11_alt_func ), .altFunc = port_a11_alt_func },
      { .pinID = 12, .afListSize = ARRAY_COUNT( port_a12_alt_func ), .altFunc = port_a12_alt_func },
      { .pinID = 13, .afListSize = ARRAY_COUNT( port_a13_alt_func ), .altFunc = port_a13_alt_func },
      { .pinID = 14, .afListSize = ARRAY_COUNT( port_a14_alt_func ), .altFunc = port_a14_alt_func },
      { .pinID = 15, .afListSize = ARRAY_COUNT( port_a15_alt_func ), .altFunc = port_a15_alt_func }
    };


    /*-------------------------------------------------------------------------------
    PORT B CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_b0_alt_func[] = {
      { .registerAltFunc = AF1_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_CH2N       },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_NSS        },
      { .registerAltFunc = AF7_USART3,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART3_CK       },
      { .registerAltFunc = AF10_QUADSPI,      .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_BK1_IO1 },
      { .registerAltFunc = AF12_COMP1,        .chimeraAltFunc = Chimera::GPIO::Alternate::COMP1_OUT       },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_EXTCLK     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT        },
    };

    static const AlternateFunc port_b1_alt_func[] = {
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM1_CH3N       },
      { .registerAltFunc = AF7_USART3,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART3_RTS_DE   },
      { .registerAltFunc = AF8_LPUART1,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPUART1_RTS_DE  },
      { .registerAltFunc = AF10_QUADSPI,      .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_BK1_IO0 },
      { .registerAltFunc = AF14_LPTIM2,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPTIM2_IN1      },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT        },
    };

    static const AlternateFunc port_b3_alt_func[] = {
      { .registerAltFunc = AF0_SWJ,           .chimeraAltFunc = Chimera::GPIO::Alternate::JTDO_TRACESWO },
      { .registerAltFunc = AF1_TIM2,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM2_CH2      },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_SCK      },
      { .registerAltFunc = AF6_SPI3,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI3_SCK      },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_RTS_DE },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_SCK_B    },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT      },
    };

    static const AlternateFunc port_b4_alt_func[] = {
      { .registerAltFunc = AF0_SWJ,           .chimeraAltFunc = Chimera::GPIO::Alternate::NJTRST      },
      { .registerAltFunc = AF4_I2C3,          .chimeraAltFunc = Chimera::GPIO::Alternate::I2C3_SDA    },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_MISO   },
      { .registerAltFunc = AF6_SPI3,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI3_MISO   },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_CTS  },
      { .registerAltFunc = AF9_TSC,           .chimeraAltFunc = Chimera::GPIO::Alternate::TSC_G2_IO1  },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_MCLK_B },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_b5_alt_func[] = {
      { .registerAltFunc = AF1_TIM1,          .chimeraAltFunc = Chimera::GPIO::Alternate::LPTIM1_IN1  },
      { .registerAltFunc = AF4_I2C1,          .chimeraAltFunc = Chimera::GPIO::Alternate::I2C1_SMBA   },
      { .registerAltFunc = AF5_SPI1,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI1_MOSI   },
      { .registerAltFunc = AF6_SPI3,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI3_MOSI   },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_CK   },
      { .registerAltFunc = AF9_TSC,           .chimeraAltFunc = Chimera::GPIO::Alternate::TSC_G2_IO2  },
      { .registerAltFunc = AF12_COMP2,        .chimeraAltFunc = Chimera::GPIO::Alternate::COMP2_OUT   },
      { .registerAltFunc = AF13_SAI1,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_SD_B   },
      { .registerAltFunc = AF14_TIM16,        .chimeraAltFunc = Chimera::GPIO::Alternate::TIM16_BKIN  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_b6_alt_func[] = {
      { .registerAltFunc =  AF1_LPTIM1,       .chimeraAltFunc = Chimera::GPIO::Alternate::LPTIM1_ETR },
      { .registerAltFunc =  AF4_I2C1,         .chimeraAltFunc = Chimera::GPIO::Alternate::I2C1_SCL   },
      { .registerAltFunc =  AF7_USART1,       .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_TX  },
      { .registerAltFunc =  AF9_TSC,          .chimeraAltFunc = Chimera::GPIO::Alternate::TSC_G2_IO3 },
      { .registerAltFunc =  AF13_SAI1,        .chimeraAltFunc = Chimera::GPIO::Alternate::SAI1_FS_B  },
      { .registerAltFunc =  AF14_TIM16,       .chimeraAltFunc = Chimera::GPIO::Alternate::TIM16_CH1N },
      { .registerAltFunc =  AF15_EVENTOUT,    .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT   }
    };

    static const AlternateFunc port_b7_alt_func[] = {
      { .registerAltFunc = AF1_LPTIM1,        .chimeraAltFunc = Chimera::GPIO::Alternate::LPTIM1_IN2  },
      { .registerAltFunc = AF4_I2C1,          .chimeraAltFunc = Chimera::GPIO::Alternate::I2C1_SDA    },
      { .registerAltFunc = AF7_USART1,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART1_RX   },
      { .registerAltFunc = AF9_TSC,           .chimeraAltFunc = Chimera::GPIO::Alternate::TSC_G2_IO4  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    }
    };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_b_pin_attributes[ GPIOB_NUM_PINS ] = {
      // Pin 2 not supported
      { .pinID = 0, .afListSize = ARRAY_COUNT( port_b0_alt_func ), .altFunc = port_b0_alt_func },
      { .pinID = 1, .afListSize = ARRAY_COUNT( port_b1_alt_func ), .altFunc = port_b1_alt_func },
      { .pinID = 3, .afListSize = ARRAY_COUNT( port_b3_alt_func ), .altFunc = port_b3_alt_func },
      { .pinID = 4, .afListSize = ARRAY_COUNT( port_b4_alt_func ), .altFunc = port_b4_alt_func },
      { .pinID = 5, .afListSize = ARRAY_COUNT( port_b5_alt_func ), .altFunc = port_b5_alt_func },
      { .pinID = 6, .afListSize = ARRAY_COUNT( port_b6_alt_func ), .altFunc = port_b6_alt_func },
      { .pinID = 7, .afListSize = ARRAY_COUNT( port_b7_alt_func ), .altFunc = port_b7_alt_func },
    };


    /*-------------------------------------------------------------------------------
    PORT C CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_c14_alt_func[] = {
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc  = Chimera::GPIO::Alternate::EVENTOUT }
    };

    static const AlternateFunc port_c15_alt_func[] = {
      { .registerAltFunc = AF15_EVENTOUT, .chimeraAltFunc  = Chimera::GPIO::Alternate::EVENTOUT }
    };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_c_pin_attributes[ GPIOC_NUM_PINS ] = {
      { .pinID = 14, .afListSize = ARRAY_COUNT( port_c14_alt_func ), .altFunc = port_c14_alt_func },
      { .pinID = 15, .afListSize = ARRAY_COUNT( port_c15_alt_func ), .altFunc = port_c15_alt_func }
    };
    /* clang-format on */
  }    // namespace Internal

  extern const PortAttributes prjPortAttributes[ NUM_GPIO_PERIPHS ] = {
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

  static_assert( NUM_GPIO_PERIPHS == ARRAY_COUNT( prjPortAttributes ) );
  static_assert( GPIOA_NUM_PINS == ARRAY_COUNT( Internal::port_a_pin_attributes ) );
  static_assert( GPIOB_NUM_PINS == ARRAY_COUNT( Internal::port_b_pin_attributes ) );
  static_assert( GPIOC_NUM_PINS == ARRAY_COUNT( Internal::port_c_pin_attributes ) );


  extern const std::uintptr_t prjPortAddress[ NUM_GPIO_PERIPHS ] = {
    reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH )
  };


  {
  }
}    // namespace Thor::LLD::GPIO

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_gpio_mapping.hpp
  ------------------------------------------------*/
  //RegisterConfig GPIO_ClockConfig[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
  //RegisterConfig GPIO_ResetConfig[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];
  //Chimera::Clock::Bus GPIO_SourceClock[ Thor::LLD::GPIO::NUM_GPIO_PERIPHS ];

  //PCC GPIOLookup = { GPIO_ClockConfig,
  //                   nullptr,
  //                   GPIO_ResetConfig,
  //                   GPIO_SourceClock,
  //                   Thor::LLD::GPIO::NUM_GPIO_PERIPHS,
  //                   Thor::LLD::GPIO::getResourceIndex };

  void GPIOInit()
  {
    using namespace Thor::LLD::GPIO;

    ///*------------------------------------------------
    //GPIO clock enable register access lookup table
    //------------------------------------------------*/
    //GPIO_ClockConfig[ GPIOA_RESOURCE_INDEX ].mask = AHB2ENR_GPIOAEN;
    //GPIO_ClockConfig[ GPIOA_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2ENR;

    //GPIO_ClockConfig[ GPIOB_RESOURCE_INDEX ].mask = AHB2ENR_GPIOBEN;
    //GPIO_ClockConfig[ GPIOB_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2ENR;

    //GPIO_ClockConfig[ GPIOC_RESOURCE_INDEX ].mask = AHB2ENR_GPIOCEN;
    //GPIO_ClockConfig[ GPIOC_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2ENR;

    ///*------------------------------------------------
    //GPIO reset register access lookup table
    //------------------------------------------------*/
    //GPIO_ResetConfig[ GPIOA_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOARST;
    //GPIO_ResetConfig[ GPIOA_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    //GPIO_ResetConfig[ GPIOB_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOBRST;
    //GPIO_ResetConfig[ GPIOB_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    //GPIO_ResetConfig[ GPIOC_RESOURCE_INDEX ].mask = AHB2RSTR_GPIOCRST;
    //GPIO_ResetConfig[ GPIOC_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB2RSTR;

    ///*------------------------------------------------
    //GPIO clocking bus source identifier
    //------------------------------------------------*/
    //GPIO_SourceClock[ GPIOA_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
    //GPIO_SourceClock[ GPIOB_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
    //GPIO_SourceClock[ GPIOC_RESOURCE_INDEX ] = Chimera::Clock::Bus::PCLK2;
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_GPIO */
