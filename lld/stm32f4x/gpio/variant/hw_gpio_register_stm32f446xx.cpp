/********************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32f446xx.cpp
 *
 *  Description:
 *    Explicit STM32F446xx GPIO data and routines
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/gpio>
#include <Thor/lld/interface/inc/rcc>

#if defined( STM32F446xx ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Pin Alternate Configuration Mapping
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
      { .registerAltFunc = AF2_TIM5,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM5_CH1    },
      { .registerAltFunc = AF3_TIM8,          .chimeraAltFunc = Chimera::GPIO::Alternate::TIM8_ETR    },
      { .registerAltFunc = AF7_USART2,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART2_CTS  },
      { .registerAltFunc = AF8_UART4,         .chimeraAltFunc = Chimera::GPIO::Alternate::UART4_TX    },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
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

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_b_pin_attributes[ GPIOB_NUM_PINS ] = {
      { .pinID = 0,  .afListSize = ARRAY_COUNT( port_b0_alt_func ),  .altFunc = port_b0_alt_func  },
      { .pinID = 1,  .afListSize = ARRAY_COUNT( port_b1_alt_func ),  .altFunc = port_b1_alt_func  },
      { .pinID = 2,  .afListSize = ARRAY_COUNT( port_b2_alt_func ),  .altFunc = port_b2_alt_func  },
      { .pinID = 3,  .afListSize = ARRAY_COUNT( port_b3_alt_func ),  .altFunc = port_b3_alt_func  },
      { .pinID = 4,  .afListSize = ARRAY_COUNT( port_b4_alt_func ),  .altFunc = port_b4_alt_func  },
      { .pinID = 5,  .afListSize = ARRAY_COUNT( port_b5_alt_func ),  .altFunc = port_b5_alt_func  },
      { .pinID = 6,  .afListSize = ARRAY_COUNT( port_b6_alt_func ),  .altFunc = port_b6_alt_func  },
      { .pinID = 7,  .afListSize = ARRAY_COUNT( port_b7_alt_func ),  .altFunc = port_b7_alt_func  },
      { .pinID = 8,  .afListSize = ARRAY_COUNT( port_b8_alt_func ),  .altFunc = port_b8_alt_func  },
      { .pinID = 9,  .afListSize = ARRAY_COUNT( port_b9_alt_func ),  .altFunc = port_b9_alt_func  },
      { .pinID = 10, .afListSize = ARRAY_COUNT( port_b10_alt_func ), .altFunc = port_b10_alt_func },
      { .pinID = 11, .afListSize = ARRAY_COUNT( port_b11_alt_func ), .altFunc = port_b11_alt_func },
      { .pinID = 12, .afListSize = ARRAY_COUNT( port_b12_alt_func ), .altFunc = port_b12_alt_func },
      { .pinID = 13, .afListSize = ARRAY_COUNT( port_b13_alt_func ), .altFunc = port_b13_alt_func },
      { .pinID = 14, .afListSize = ARRAY_COUNT( port_b14_alt_func ), .altFunc = port_b14_alt_func },
      { .pinID = 15, .afListSize = ARRAY_COUNT( port_b15_alt_func ), .altFunc = port_b15_alt_func }
    };

    /*-------------------------------------------------------------------------------
    PORT C CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_c_pin_attributes[ GPIOC_NUM_PINS ] = {
      { .pinID = 0,  .afListSize = ARRAY_COUNT( port_c0_alt_func ),  .altFunc = port_c0_alt_func  },
      { .pinID = 1,  .afListSize = ARRAY_COUNT( port_c1_alt_func ),  .altFunc = port_c1_alt_func  },
      { .pinID = 2,  .afListSize = ARRAY_COUNT( port_c2_alt_func ),  .altFunc = port_c2_alt_func  },
      { .pinID = 3,  .afListSize = ARRAY_COUNT( port_c3_alt_func ),  .altFunc = port_c3_alt_func  },
      { .pinID = 4,  .afListSize = ARRAY_COUNT( port_c4_alt_func ),  .altFunc = port_c4_alt_func  },
      { .pinID = 5,  .afListSize = ARRAY_COUNT( port_c5_alt_func ),  .altFunc = port_c5_alt_func  },
      { .pinID = 6,  .afListSize = ARRAY_COUNT( port_c6_alt_func ),  .altFunc = port_c6_alt_func  },
      { .pinID = 7,  .afListSize = ARRAY_COUNT( port_c7_alt_func ),  .altFunc = port_c7_alt_func  },
      { .pinID = 8,  .afListSize = ARRAY_COUNT( port_c8_alt_func ),  .altFunc = port_c8_alt_func  },
      { .pinID = 9,  .afListSize = ARRAY_COUNT( port_c9_alt_func ),  .altFunc = port_c9_alt_func  },
      { .pinID = 10, .afListSize = ARRAY_COUNT( port_c10_alt_func ), .altFunc = port_c10_alt_func },
      { .pinID = 11, .afListSize = ARRAY_COUNT( port_c11_alt_func ), .altFunc = port_c11_alt_func },
      { .pinID = 12, .afListSize = ARRAY_COUNT( port_c12_alt_func ), .altFunc = port_c12_alt_func },
      { .pinID = 13, .afListSize = ARRAY_COUNT( port_c13_alt_func ), .altFunc = port_c13_alt_func },
      { .pinID = 14, .afListSize = ARRAY_COUNT( port_c14_alt_func ), .altFunc = port_c14_alt_func },
      { .pinID = 15, .afListSize = ARRAY_COUNT( port_c15_alt_func ), .altFunc = port_c15_alt_func }
    };

    /*-------------------------------------------------------------------------------
    PORT D CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_d_pin_attributes[ GPIOD_NUM_PINS ] = {
      { .pinID = 0,  .afListSize = ARRAY_COUNT( port_d0_alt_func ),  .altFunc = port_d0_alt_func  },
      { .pinID = 1,  .afListSize = ARRAY_COUNT( port_d1_alt_func ),  .altFunc = port_d1_alt_func  },
      { .pinID = 2,  .afListSize = ARRAY_COUNT( port_d2_alt_func ),  .altFunc = port_d2_alt_func  },
      { .pinID = 3,  .afListSize = ARRAY_COUNT( port_d3_alt_func ),  .altFunc = port_d3_alt_func  },
      { .pinID = 4,  .afListSize = ARRAY_COUNT( port_d4_alt_func ),  .altFunc = port_d4_alt_func  },
      { .pinID = 5,  .afListSize = ARRAY_COUNT( port_d5_alt_func ),  .altFunc = port_d5_alt_func  },
      { .pinID = 6,  .afListSize = ARRAY_COUNT( port_d6_alt_func ),  .altFunc = port_d6_alt_func  },
      { .pinID = 7,  .afListSize = ARRAY_COUNT( port_d7_alt_func ),  .altFunc = port_d7_alt_func  },
      { .pinID = 8,  .afListSize = ARRAY_COUNT( port_d8_alt_func ),  .altFunc = port_d8_alt_func  },
      { .pinID = 9,  .afListSize = ARRAY_COUNT( port_d9_alt_func ),  .altFunc = port_d9_alt_func  },
      { .pinID = 10, .afListSize = ARRAY_COUNT( port_d10_alt_func ), .altFunc = port_d10_alt_func },
      { .pinID = 11, .afListSize = ARRAY_COUNT( port_d11_alt_func ), .altFunc = port_d11_alt_func },
      { .pinID = 12, .afListSize = ARRAY_COUNT( port_d12_alt_func ), .altFunc = port_d12_alt_func },
      { .pinID = 13, .afListSize = ARRAY_COUNT( port_d13_alt_func ), .altFunc = port_d13_alt_func },
      { .pinID = 14, .afListSize = ARRAY_COUNT( port_d14_alt_func ), .altFunc = port_d14_alt_func },
      { .pinID = 15, .afListSize = ARRAY_COUNT( port_d15_alt_func ), .altFunc = port_d15_alt_func }
    };

    /*-------------------------------------------------------------------------------
    PORT E CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_e_pin_attributes[ GPIOE_NUM_PINS ] = {
      { .pinID = 0,  .afListSize = ARRAY_COUNT( port_e0_alt_func ),  .altFunc = port_e0_alt_func  },
      { .pinID = 1,  .afListSize = ARRAY_COUNT( port_e1_alt_func ),  .altFunc = port_e1_alt_func  },
      { .pinID = 2,  .afListSize = ARRAY_COUNT( port_e2_alt_func ),  .altFunc = port_e2_alt_func  },
      { .pinID = 3,  .afListSize = ARRAY_COUNT( port_e3_alt_func ),  .altFunc = port_e3_alt_func  },
      { .pinID = 4,  .afListSize = ARRAY_COUNT( port_e4_alt_func ),  .altFunc = port_e4_alt_func  },
      { .pinID = 5,  .afListSize = ARRAY_COUNT( port_e5_alt_func ),  .altFunc = port_e5_alt_func  },
      { .pinID = 6,  .afListSize = ARRAY_COUNT( port_e6_alt_func ),  .altFunc = port_e6_alt_func  },
      { .pinID = 7,  .afListSize = ARRAY_COUNT( port_e7_alt_func ),  .altFunc = port_e7_alt_func  },
      { .pinID = 8,  .afListSize = ARRAY_COUNT( port_e8_alt_func ),  .altFunc = port_e8_alt_func  },
      { .pinID = 9,  .afListSize = ARRAY_COUNT( port_e9_alt_func ),  .altFunc = port_e9_alt_func  },
      { .pinID = 10, .afListSize = ARRAY_COUNT( port_e10_alt_func ), .altFunc = port_e10_alt_func },
      { .pinID = 11, .afListSize = ARRAY_COUNT( port_e11_alt_func ), .altFunc = port_e11_alt_func },
      { .pinID = 12, .afListSize = ARRAY_COUNT( port_e12_alt_func ), .altFunc = port_e12_alt_func },
      { .pinID = 13, .afListSize = ARRAY_COUNT( port_e13_alt_func ), .altFunc = port_e13_alt_func },
      { .pinID = 14, .afListSize = ARRAY_COUNT( port_e14_alt_func ), .altFunc = port_e14_alt_func },
      { .pinID = 15, .afListSize = ARRAY_COUNT( port_e15_alt_func ), .altFunc = port_e15_alt_func }
    };

    /*-------------------------------------------------------------------------------
    PORT F CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_f_pin_attributes[ GPIOF_NUM_PINS ] = {
      { .pinID = 0,  .afListSize = ARRAY_COUNT( port_f0_alt_func ),  .altFunc = port_f0_alt_func  },
      { .pinID = 1,  .afListSize = ARRAY_COUNT( port_f1_alt_func ),  .altFunc = port_f1_alt_func  },
      { .pinID = 2,  .afListSize = ARRAY_COUNT( port_f2_alt_func ),  .altFunc = port_f2_alt_func  },
      { .pinID = 3,  .afListSize = ARRAY_COUNT( port_f3_alt_func ),  .altFunc = port_f3_alt_func  },
      { .pinID = 4,  .afListSize = ARRAY_COUNT( port_f4_alt_func ),  .altFunc = port_f4_alt_func  },
      { .pinID = 5,  .afListSize = ARRAY_COUNT( port_f5_alt_func ),  .altFunc = port_f5_alt_func  },
      { .pinID = 6,  .afListSize = ARRAY_COUNT( port_f6_alt_func ),  .altFunc = port_f6_alt_func  },
      { .pinID = 7,  .afListSize = ARRAY_COUNT( port_f7_alt_func ),  .altFunc = port_f7_alt_func  },
      { .pinID = 8,  .afListSize = ARRAY_COUNT( port_f8_alt_func ),  .altFunc = port_f8_alt_func  },
      { .pinID = 9,  .afListSize = ARRAY_COUNT( port_f9_alt_func ),  .altFunc = port_f9_alt_func  },
      { .pinID = 10, .afListSize = ARRAY_COUNT( port_f10_alt_func ), .altFunc = port_f10_alt_func },
      { .pinID = 11, .afListSize = ARRAY_COUNT( port_f11_alt_func ), .altFunc = port_f11_alt_func },
      { .pinID = 12, .afListSize = ARRAY_COUNT( port_f12_alt_func ), .altFunc = port_f12_alt_func },
      { .pinID = 13, .afListSize = ARRAY_COUNT( port_f13_alt_func ), .altFunc = port_f13_alt_func },
      { .pinID = 14, .afListSize = ARRAY_COUNT( port_f14_alt_func ), .altFunc = port_f14_alt_func },
      { .pinID = 15, .afListSize = ARRAY_COUNT( port_f15_alt_func ), .altFunc = port_f15_alt_func }
    };

    /*-------------------------------------------------------------------------------
    PORT G CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_g0_alt_func[] = {
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_A10     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g1_alt_func[] = {
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_A11     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g2_alt_func[] = {
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_A12     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g3_alt_func[] = {
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_A13     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g4_alt_func[] = {
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_A14     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g5_alt_func[] = {
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_A15     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g6_alt_func[] = {
      { .registerAltFunc = AF10_QSPI,         .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_BK1_NCS },
      { .registerAltFunc = AF13_DCMI,         .chimeraAltFunc = Chimera::GPIO::Alternate::DCMI_D12    },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g7_alt_func[] = {
      { .registerAltFunc = AF8_USART6,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART6_CK   },
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_INT     },
      { .registerAltFunc = AF13_DCMI,         .chimeraAltFunc = Chimera::GPIO::Alternate::DCMI_D13    },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g8_alt_func[] = {
      { .registerAltFunc = AF7_SPDIFRX,       .chimeraAltFunc = Chimera::GPIO::Alternate::SPDIFRX_IN2 },
      { .registerAltFunc = AF8_USART6,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART6_RTS  },
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_SDCLK   },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g9_alt_func[] = {
      { .registerAltFunc = AF7_SPDIFRX,       .chimeraAltFunc = Chimera::GPIO::Alternate::SPDIFRX_IN3 },
      { .registerAltFunc = AF8_USART6,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART6_RX   },
      { .registerAltFunc = AF9_QSPI,          .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_BK2_IO2 },
      { .registerAltFunc = AF10_SAI2,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI2_FS_B   },
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_NE2     },
      { .registerAltFunc = AF13_DCMI,         .chimeraAltFunc = Chimera::GPIO::Alternate::DCMI_VSYNC  },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g10_alt_func[] = {
      { .registerAltFunc = AF10_SAI2,         .chimeraAltFunc = Chimera::GPIO::Alternate::SAI2_SD_B   },
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_NE3     },
      { .registerAltFunc = AF13_DCMI,         .chimeraAltFunc = Chimera::GPIO::Alternate::DCMI_D2     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g11_alt_func[] = {
      { .registerAltFunc = AF6_SPI4,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI4_SCK    },
      { .registerAltFunc = AF7_SPDIFRX,       .chimeraAltFunc = Chimera::GPIO::Alternate::SPDIFRX_IN0 },
      { .registerAltFunc = AF13_DCMI,         .chimeraAltFunc = Chimera::GPIO::Alternate::DCMI_D3     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g12_alt_func[] = {
      { .registerAltFunc = AF6_SPI4,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI4_MISO   },
      { .registerAltFunc = AF7_SPDIFRX,       .chimeraAltFunc = Chimera::GPIO::Alternate::SPDIFRX_IN1 },
      { .registerAltFunc = AF8_USART6,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART6_RTS  },
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_NE4     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g13_alt_func[] = {
      { .registerAltFunc = AF0_TRACE,         .chimeraAltFunc = Chimera::GPIO::Alternate::TRACE_D2    },
      { .registerAltFunc = AF6_SPI4,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI4_MOSI   },
      { .registerAltFunc = AF8_USART6,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART6_CTS  },
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_A24     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g14_alt_func[] = {
      { .registerAltFunc = AF0_TRACE,         .chimeraAltFunc = Chimera::GPIO::Alternate::TRACE_D3    },
      { .registerAltFunc = AF6_SPI4,          .chimeraAltFunc = Chimera::GPIO::Alternate::SPI4_NSS    },
      { .registerAltFunc = AF8_USART6,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART6_TX   },
      { .registerAltFunc = AF9_QSPI,          .chimeraAltFunc = Chimera::GPIO::Alternate::QUADSPI_BK2_IO3 },
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_A25     },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_g15_alt_func[] = {
      { .registerAltFunc = AF8_USART6,        .chimeraAltFunc = Chimera::GPIO::Alternate::USART6_CTS  },
      { .registerAltFunc = AF12_FMC,          .chimeraAltFunc = Chimera::GPIO::Alternate::FMC_SDNCAS  },
      { .registerAltFunc = AF13_DCMI,         .chimeraAltFunc = Chimera::GPIO::Alternate::DCMI_D13    },
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_g_pin_attributes[ GPIOG_NUM_PINS ] = {
      { .pinID = 0,  .afListSize = ARRAY_COUNT( port_g0_alt_func ),  .altFunc = port_g0_alt_func  },
      { .pinID = 1,  .afListSize = ARRAY_COUNT( port_g1_alt_func ),  .altFunc = port_g1_alt_func  },
      { .pinID = 2,  .afListSize = ARRAY_COUNT( port_g2_alt_func ),  .altFunc = port_g2_alt_func  },
      { .pinID = 3,  .afListSize = ARRAY_COUNT( port_g3_alt_func ),  .altFunc = port_g3_alt_func  },
      { .pinID = 4,  .afListSize = ARRAY_COUNT( port_g4_alt_func ),  .altFunc = port_g4_alt_func  },
      { .pinID = 5,  .afListSize = ARRAY_COUNT( port_g5_alt_func ),  .altFunc = port_g5_alt_func  },
      { .pinID = 6,  .afListSize = ARRAY_COUNT( port_g6_alt_func ),  .altFunc = port_g6_alt_func  },
      { .pinID = 7,  .afListSize = ARRAY_COUNT( port_g7_alt_func ),  .altFunc = port_g7_alt_func  },
      { .pinID = 8,  .afListSize = ARRAY_COUNT( port_g8_alt_func ),  .altFunc = port_g8_alt_func  },
      { .pinID = 9,  .afListSize = ARRAY_COUNT( port_g9_alt_func ),  .altFunc = port_g9_alt_func  },
      { .pinID = 10, .afListSize = ARRAY_COUNT( port_g10_alt_func ), .altFunc = port_g10_alt_func },
      { .pinID = 11, .afListSize = ARRAY_COUNT( port_g11_alt_func ), .altFunc = port_g11_alt_func },
      { .pinID = 12, .afListSize = ARRAY_COUNT( port_g12_alt_func ), .altFunc = port_g12_alt_func },
      { .pinID = 13, .afListSize = ARRAY_COUNT( port_g13_alt_func ), .altFunc = port_g13_alt_func },
      { .pinID = 14, .afListSize = ARRAY_COUNT( port_g14_alt_func ), .altFunc = port_g14_alt_func },
      { .pinID = 15, .afListSize = ARRAY_COUNT( port_g15_alt_func ), .altFunc = port_g15_alt_func }
    };

    /*-------------------------------------------------------------------------------
    PORT H CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_h0_alt_func[] = {
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    static const AlternateFunc port_h1_alt_func[] = {
      { .registerAltFunc = AF15_EVENTOUT,     .chimeraAltFunc = Chimera::GPIO::Alternate::EVENTOUT    },
    };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_h_pin_attributes[ GPIOH_NUM_PINS ] = {
      { .pinID = 0,  .afListSize = ARRAY_COUNT( port_h0_alt_func ),  .altFunc = port_h0_alt_func  },
      { .pinID = 1,  .afListSize = ARRAY_COUNT( port_h1_alt_func ),  .altFunc = port_h1_alt_func  }
    };
  } /* clang-format on */

  /*-------------------------------------------------------------------------------
  Attribute Mapping
  -------------------------------------------------------------------------------*/
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
    },

    // GPIO D
    {
      .portID      = Chimera::GPIO::Port::PORTD,
      .pinListSize = GPIOD_NUM_PINS,
      .pins        = Internal::port_d_pin_attributes
    },

    // GPIO E
    {
      .portID      = Chimera::GPIO::Port::PORTE,
      .pinListSize = GPIOE_NUM_PINS,
      .pins        = Internal::port_e_pin_attributes
    },

    // GPIO F
    {
      .portID      = Chimera::GPIO::Port::PORTF,
      .pinListSize = GPIOF_NUM_PINS,
      .pins        = Internal::port_f_pin_attributes
    },

    // GPIO G
    {
      .portID      = Chimera::GPIO::Port::PORTG,
      .pinListSize = GPIOG_NUM_PINS,
      .pins        = Internal::port_g_pin_attributes
    },

    // GPIO H
    {
      .portID      = Chimera::GPIO::Port::PORTH,
      .pinListSize = GPIOH_NUM_PINS,
      .pins        = Internal::port_h_pin_attributes
    },
  };

  static_assert( NUM_GPIO_PERIPHS == ARRAY_COUNT( prjPortAttributes ) );
  static_assert( GPIOA_NUM_PINS == ARRAY_COUNT( Internal::port_a_pin_attributes ) );
  static_assert( GPIOB_NUM_PINS == ARRAY_COUNT( Internal::port_b_pin_attributes ) );
  static_assert( GPIOC_NUM_PINS == ARRAY_COUNT( Internal::port_c_pin_attributes ) );
  static_assert( GPIOD_NUM_PINS == ARRAY_COUNT( Internal::port_d_pin_attributes ) );
  static_assert( GPIOE_NUM_PINS == ARRAY_COUNT( Internal::port_e_pin_attributes ) );
  static_assert( GPIOF_NUM_PINS == ARRAY_COUNT( Internal::port_f_pin_attributes ) );
  static_assert( GPIOG_NUM_PINS == ARRAY_COUNT( Internal::port_g_pin_attributes ) );
  static_assert( GPIOH_NUM_PINS == ARRAY_COUNT( Internal::port_h_pin_attributes ) );


  extern const std::uintptr_t prjPortAddress[ NUM_GPIO_PERIPHS ] = {
    reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ),
    reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ),
  };
}    // namespace Thor::LLD::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
