/******************************************************************************
 *  File Name:
 *    hw_rcc_data.cpp
 *
 *  Description:
 *    Implements the required LLD data interface
 *
 *  2021-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/cfg>
#include <Thor/lld/interface/inc/adc>
#include <Thor/lld/interface/inc/can>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/flash>
#include <Thor/lld/interface/inc/gpio>
#include <Thor/lld/interface/inc/i2c>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/sdio>
#include <Thor/lld/interface/inc/spi>
#include <Thor/lld/interface/inc/sys>
#include <Thor/lld/interface/inc/timer>
#include <Thor/lld/interface/inc/uart>
#include <Thor/lld/interface/inc/usart>
#include <Thor/lld/interface/inc/watchdog>


#if defined( TARGET_STM32F4 ) && defined( THOR_RCC )

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  Peripheral Register Mappings
  ---------------------------------------------------------------------------*/
#if defined( THOR_ADC )
  /* clang-format off */
  static const RegisterConfig ADC_ClockConfig[ ADC::NUM_ADC_PERIPHS ]      = {
    { .mask = APB2ENR_ADC1EN, .reg  = &RCC1_PERIPH->APB2ENR },
    { .mask = APB2ENR_ADC2EN, .reg  = &RCC1_PERIPH->APB2ENR },
    { .mask = APB2ENR_ADC3EN, .reg  = &RCC1_PERIPH->APB2ENR }
  };

  static const RegisterConfig ADC_ResetConfig[ ADC::NUM_ADC_PERIPHS ]      = {
    { .mask = APB2RSTR_ADCRST, .reg = &RCC1_PERIPH->APB2RSTR },
    { .mask = APB2RSTR_ADCRST, .reg = &RCC1_PERIPH->APB2RSTR },
    { .mask = APB2RSTR_ADCRST, .reg = &RCC1_PERIPH->APB2RSTR }
  };

  static const Chimera::Clock::Bus ADC_SourceClock[ ADC::NUM_ADC_PERIPHS ] = {
    Chimera::Clock::Bus::PCLK2,
    Chimera::Clock::Bus::PCLK2,
    Chimera::Clock::Bus::PCLK2
  };
  /* clang-format on */
#endif /* THOR_LLD_ADC */

#if defined( THOR_CAN )
  /* clang-format off */
  static const RegisterConfig CAN_ClockConfig[ CAN::NUM_CAN_PERIPHS ]      = {
    { .mask = APB1ENR_CAN1EN, .reg  = &RCC1_PERIPH->APB1ENR },
    { .mask = APB1ENR_CAN2EN, .reg  = &RCC1_PERIPH->APB1ENR },
  };

  static const RegisterConfig CAN_ResetConfig[ CAN::NUM_CAN_PERIPHS ]      = {
    { .mask = APB1RSTR_CAN1RST, .reg = &RCC1_PERIPH->APB1RSTR },
    { .mask = APB1RSTR_CAN2RST, .reg = &RCC1_PERIPH->APB1RSTR },
  };

  static const Chimera::Clock::Bus CAN_SourceClock[ CAN::NUM_CAN_PERIPHS ] = {
    Chimera::Clock::Bus::APB1,
    Chimera::Clock::Bus::APB1
  };
  /* clang-format on */
#endif /* THOR_LLD_CAN */

#if defined( THOR_CRS )
  // /* clang-format off */
  // static const RegisterConfig CRS_ClockConfig[ CRS::NUM_CRS_PERIPHS ]      = {
  //   { .mask = APB1ENR1_CRSEN, .reg  = &RCC1_PERIPH->APB1ENR1 }
  // };

  // static const RegisterConfig CRS_ResetConfig[ CRS::NUM_CRS_PERIPHS ]      = {
  //   { .mask = APB1RSTR1_CRSRST, .reg = &RCC1_PERIPH->APB1RSTR1 }
  // };

  // static const Chimera::Clock::Bus CRS_SourceClock[ CRS::NUM_CRS_PERIPHS ] = {
  //   Chimera::Clock::Bus::APB1
  // };
  // /* clang-format on */
#endif /* THOR_LLD_CRS */

#if defined( THOR_DMA )
  /* clang-format off */
  static const RegisterConfig DMA_ClockConfig[ DMA::NUM_DMA_PERIPHS ]      = {
    { .mask = AHB1ENR_DMA1EN, .reg  = &RCC1_PERIPH->AHB1ENR },
    { .mask = AHB1ENR_DMA2EN, .reg  = &RCC1_PERIPH->AHB1ENR }
  };

  static const RegisterConfig DMA_ResetConfig[ DMA::NUM_DMA_PERIPHS ]      = {
    { .mask = AHB1RSTR_DMA1RST, .reg = &RCC1_PERIPH->AHB1RSTR },
    { .mask = AHB1RSTR_DMA2RST, .reg = &RCC1_PERIPH->AHB1RSTR }
  };

  static const Chimera::Clock::Bus DMA_SourceClock[ DMA::NUM_DMA_PERIPHS ] = {
    Chimera::Clock::Bus::HCLK,
    Chimera::Clock::Bus::HCLK
  };
  /* clang-format on */
#endif /* THOR_LLD_DMA */

#if defined( THOR_FLASH )
  /* clang-format off */
  static const RegisterConfig FLASH_ClockConfig[ FLASH::NUM_FLASH_PERIPHS ]      = {
    { .mask = CKGATENR_FLITF_CKEN, .reg  = &RCC1_PERIPH->CKGATENR }
  };

  static const RegisterConfig FLASH_ResetConfig[ FLASH::NUM_FLASH_PERIPHS ]      = {
    { .mask = 0, .reg = nullptr }
  };

  static const Chimera::Clock::Bus FLASH_SourceClock[ FLASH::NUM_FLASH_PERIPHS ] = {
    Chimera::Clock::Bus::AHB
  };
  /* clang-format on */
#endif /* THOR_LLD_FLASH */

#if defined( THOR_GPIO )
  /* clang-format off */
  static const RegisterConfig GPIO_ClockConfig[ GPIO::NUM_GPIO_PERIPHS ]      = {
    #if defined( STM32_GPIOA_PERIPH_AVAILABLE )
    { .mask = AHB1ENR_GPIOAEN, .reg  = &RCC1_PERIPH->AHB1ENR },
    #endif

    #if defined( STM32_GPIOB_PERIPH_AVAILABLE )
    { .mask = AHB1ENR_GPIOBEN, .reg  = &RCC1_PERIPH->AHB1ENR },
    #endif

    #if defined( STM32_GPIOC_PERIPH_AVAILABLE )
    { .mask = AHB1ENR_GPIOCEN, .reg  = &RCC1_PERIPH->AHB1ENR },
    #endif

    #if defined( STM32_GPIOD_PERIPH_AVAILABLE )
    { .mask = AHB1ENR_GPIODEN, .reg  = &RCC1_PERIPH->AHB1ENR },
    #endif

    #if defined( STM32_GPIOE_PERIPH_AVAILABLE )
    { .mask = AHB1ENR_GPIOEEN, .reg  = &RCC1_PERIPH->AHB1ENR },
    #endif

    #if defined( STM32_GPIOF_PERIPH_AVAILABLE )
    { .mask = AHB1ENR_GPIOFEN, .reg  = &RCC1_PERIPH->AHB1ENR },
    #endif

    #if defined( STM32_GPIOG_PERIPH_AVAILABLE )
    { .mask = AHB1ENR_GPIOGEN, .reg  = &RCC1_PERIPH->AHB1ENR },
    #endif

    #if defined( STM32_GPIOH_PERIPH_AVAILABLE )
    { .mask = AHB1ENR_GPIOHEN, .reg  = &RCC1_PERIPH->AHB1ENR },
    #endif
  };

  static const RegisterConfig GPIO_ResetConfig[ GPIO::NUM_GPIO_PERIPHS ]      = {
    #if defined( STM32_GPIOA_PERIPH_AVAILABLE )
    { .mask = AHB1RSTR_GPIOARST, .reg = &RCC1_PERIPH->AHB1RSTR },
    #endif

    #if defined( STM32_GPIOB_PERIPH_AVAILABLE )
    { .mask = AHB1RSTR_GPIOBRST, .reg = &RCC1_PERIPH->AHB1RSTR },
    #endif

    #if defined( STM32_GPIOC_PERIPH_AVAILABLE )
    { .mask = AHB1RSTR_GPIOCRST, .reg = &RCC1_PERIPH->AHB1RSTR },
    #endif

    #if defined( STM32_GPIOD_PERIPH_AVAILABLE )
    { .mask = AHB1RSTR_GPIODRST, .reg = &RCC1_PERIPH->AHB1RSTR },
    #endif

    #if defined( STM32_GPIOE_PERIPH_AVAILABLE )
    { .mask = AHB1RSTR_GPIOERST, .reg = &RCC1_PERIPH->AHB1RSTR },
    #endif

    #if defined( STM32_GPIOF_PERIPH_AVAILABLE )
    { .mask = AHB1RSTR_GPIOFRST, .reg = &RCC1_PERIPH->AHB1RSTR },
    #endif

    #if defined( STM32_GPIOG_PERIPH_AVAILABLE )
    { .mask = AHB1RSTR_GPIOGRST, .reg = &RCC1_PERIPH->AHB1RSTR },
    #endif

    #if defined( STM32_GPIOH_PERIPH_AVAILABLE )
    { .mask = AHB1RSTR_GPIOHRST, .reg = &RCC1_PERIPH->AHB1RSTR },
    #endif
  };

  static const Chimera::Clock::Bus GPIO_SourceClock[ GPIO::NUM_GPIO_PERIPHS ] = {
    #if defined( STM32_GPIOA_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_GPIOB_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_GPIOC_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_GPIOD_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_GPIOE_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_GPIOF_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_GPIOG_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_GPIOH_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif
  };
  /* clang-format on */
#endif /* THOR_LLD_GPIO */

#if defined( THOR_I2C )
  /* clang-format off */
  static const RegisterConfig I2C_ClockConfig[ I2C::NUM_I2C_PERIPHS ]      = {
    { .mask = APB1ENR_I2C1EN, .reg  = &RCC1_PERIPH->APB1ENR },
    { .mask = APB1ENR_I2C2EN, .reg  = &RCC1_PERIPH->APB1ENR },
    { .mask = APB1ENR_I2C3EN, .reg  = &RCC1_PERIPH->APB1ENR }
  };

  static const RegisterConfig I2C_ResetConfig[ I2C::NUM_I2C_PERIPHS ]      = {
    { .mask = APB1RSTR_I2C1RST, .reg = &RCC1_PERIPH->APB1RSTR },
    { .mask = APB1RSTR_I2C2RST, .reg = &RCC1_PERIPH->APB1RSTR },
    { .mask = APB1RSTR_I2C3RST, .reg = &RCC1_PERIPH->APB1RSTR }
  };

  static const Chimera::Clock::Bus I2C_SourceClock[ I2C::NUM_I2C_PERIPHS ] = {
    Chimera::Clock::Bus::APB1,
    Chimera::Clock::Bus::APB1,
    Chimera::Clock::Bus::APB1
  };
  /* clang-format on */
#endif /* THOR_LLD_I2C */

#if defined( THOR_PWR )
  /* clang-format off */
  static const RegisterConfig PWR_ClockConfig[ PWR::NUM_PWR_PERIPHS ]      = {
    { .mask = APB1ENR_PWREN, .reg  = &RCC1_PERIPH->APB1ENR }
  };

  static const RegisterConfig PWR_ResetConfig[ PWR::NUM_PWR_PERIPHS ]      = {
    { .mask = APB1RSTR_PWRRST, .reg = &RCC1_PERIPH->APB1RSTR }
  };

  static const Chimera::Clock::Bus PWR_SourceClock[ PWR::NUM_PWR_PERIPHS ] = {
    Chimera::Clock::Bus::APB1
  };
  /* clang-format on */
#endif /* THOR_LLD_PWR */

#if defined( THOR_SDIO )
  /* clang-format off */
  static const RegisterConfig SDIO_ClockConfig[ SDIO::NUM_SDIO_PERIPHS ] = {
    { .mask = APB2ENR_SDIOEN, .reg  = &RCC1_PERIPH->APB2ENR }
  };

  static const RegisterConfig SDIO_ResetConfig[ SDIO::NUM_SDIO_PERIPHS ] = {
    { .mask = APB2RSTR_SDIORST, .reg = &RCC1_PERIPH->APB2RSTR }
  };

  static const Chimera::Clock::Bus SDIO_SourceClock[ SDIO::NUM_SDIO_PERIPHS ] = {
    Chimera::Clock::Bus::APB2
  };
  /* clang-format on */
#endif /* THOR_LLD_SDIO */

#if defined( THOR_SPI )
  /* clang-format off */
  static const RegisterConfig SPI_ClockConfig[ SPI::NUM_SPI_PERIPHS ]      = {
    #if defined( STM32_SPI1_PERIPH_AVAILABLE )
    { .mask = APB2ENR_SPI1EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif

    #if defined( STM32_SPI2_PERIPH_AVAILABLE )
    { .mask = APB1ENR_SPI2EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif

    #if defined( STM32_SPI3_PERIPH_AVAILABLE )
    { .mask = APB1ENR_SPI3EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif

    #if defined( STM32_SPI4_PERIPH_AVAILABLE )
    { .mask = APB2ENR_SPI4EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif
  };

  static const RegisterConfig SPI_ResetConfig[ SPI::NUM_SPI_PERIPHS ]      = {
    #if defined( STM32_SPI1_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_SPI1RST, .reg = &RCC1_PERIPH->APB2RSTR },
    #endif

    #if defined( STM32_SPI2_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_SPI2RST, .reg = &RCC1_PERIPH->APB1RSTR },
    #endif

    #if defined( STM32_SPI3_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_SPI3RST, .reg = &RCC1_PERIPH->APB1RSTR },
    #endif

    #if defined( STM32_SPI4_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_SPI4RST, .reg = &RCC1_PERIPH->APB2RSTR },
    #endif
  };

  static const Chimera::Clock::Bus SPI_SourceClock[ SPI::NUM_SPI_PERIPHS ] = {
    #if defined( STM32_SPI1_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK2,
    #endif

    #if defined( STM32_SPI2_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_SPI3_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_SPI4_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK2,
    #endif
  };
  /* clang-format on */
#endif /* THOR_LLD_SPI */

#if defined( THOR_SYSCFG )
  /* clang-format off */
  static const RegisterConfig SYSCFG_ClockConfig[ SYS::NUM_SYSCFG_PERIPHS ]      = {
    { .mask = APB2ENR_SYSCFGEN, .reg  = &RCC1_PERIPH->APB2ENR }

  };

  static const RegisterConfig SYSCFG_ResetConfig[ SYS::NUM_SYSCFG_PERIPHS ]      = {
    { .mask = APB2RSTR_SYSCFGRST, .reg = &RCC1_PERIPH->APB2RSTR }
  };

  static const Chimera::Clock::Bus SYSCFG_SourceClock[ SYS::NUM_SYSCFG_PERIPHS ] = {
    Chimera::Clock::Bus::APB2
  };
  /* clang-format on */
#endif /* THOR_LLD_SYSCFG */

#if defined( THOR_TIMER )
  /* clang-format off */
  static const RegisterConfig TIMER_ClockConfig[ TIMER::NUM_TIMER_PERIPHS ] = {
    #if defined( STM32_TIMER1_PERIPH_AVAILABLE )
    { .mask = APB2ENR_TIM1EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif
    #if defined( STM32_TIMER2_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM2EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
    #if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM3EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
    #if defined( STM32_TIMER4_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM4EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
    #if defined( STM32_TIMER5_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM5EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
    #if defined( STM32_TIMER6_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM6EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
    #if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM7EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
    #if defined( STM32_TIMER8_PERIPH_AVAILABLE )
    { .mask = APB2ENR_TIM8EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif
    #if defined( STM32_TIMER9_PERIPH_AVAILABLE )
    { .mask = APB2ENR_TIM9EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif
    #if defined( STM32_TIMER10_PERIPH_AVAILABLE )
    { .mask = APB2ENR_TIM10EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif
    #if defined( STM32_TIMER11_PERIPH_AVAILABLE )
    { .mask = APB2ENR_TIM11EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif
    #if defined( STM32_TIMER12_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM12EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
    #if defined( STM32_TIMER13_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM13EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
    #if defined( STM32_TIMER14_PERIPH_AVAILABLE )
    { .mask = APB1ENR_TIM14EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
  };

  static const RegisterConfig TIMER_ResetConfig[ TIMER::NUM_TIMER_PERIPHS ] = {
    #if defined( STM32_TIMER1_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_TIM1RST, .reg  = &RCC1_PERIPH->APB2RSTR },
    #endif
    #if defined( STM32_TIMER2_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM2RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
    #if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM3RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
    #if defined( STM32_TIMER4_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM4RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
    #if defined( STM32_TIMER5_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM5RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
    #if defined( STM32_TIMER6_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM6RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
    #if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM7RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
    #if defined( STM32_TIMER8_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_TIM8RST, .reg = &RCC1_PERIPH->APB2RSTR },
    #endif
    #if defined( STM32_TIMER9_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_TIM9RST, .reg = &RCC1_PERIPH->APB2RSTR },
    #endif
    #if defined( STM32_TIMER10_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_TIM10RST, .reg = &RCC1_PERIPH->APB2RSTR },
    #endif
    #if defined( STM32_TIMER11_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_TIM11RST, .reg = &RCC1_PERIPH->APB2RSTR },
    #endif
    #if defined( STM32_TIMER12_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM12RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
    #if defined( STM32_TIMER13_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM13RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
    #if defined( STM32_TIMER14_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_TIM14RST, .reg  = &RCC1_PERIPH->APB1RSTR },
    #endif
  };

  static const Chimera::Clock::Bus TIMER_SourceClock[ TIMER::NUM_TIMER_PERIPHS ] = {
    #if defined( STM32_TIMER1_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB2,
    #endif
    #if defined( STM32_TIMER2_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
    #if defined( STM32_TIMER3_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
    #if defined( STM32_TIMER4_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
    #if defined( STM32_TIMER5_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
    #if defined( STM32_TIMER6_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
    #if defined( STM32_TIMER7_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
    #if defined( STM32_TIMER8_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB2,
    #endif
    #if defined( STM32_TIMER9_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB2,
    #endif
    #if defined( STM32_TIMER10_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB2,
    #endif
    #if defined( STM32_TIMER11_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB2,
    #endif
    #if defined( STM32_TIMER12_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
    #if defined( STM32_TIMER13_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
    #if defined( STM32_TIMER14_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::APB1,
    #endif
  };
  /* clang-format on */
#endif /* THOR_LLD_TIMER */

#if defined( THOR_UART )
  /* clang-format off */
  static const RegisterConfig UART_ClockConfig[ UART::NUM_UART_PERIPHS ]      = {
    #if defined( STM32_UART4_PERIPH_AVAILABLE )
    { .mask = APB1ENR_UART4EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif

    #if defined( STM32_UART5_PERIPH_AVAILABLE )
    { .mask = APB1ENR_UART5EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif
  };

  static const RegisterConfig UART_ResetConfig[ UART::NUM_UART_PERIPHS ]      = {
    #if defined( STM32_UART4_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_UART4RST, .reg = &RCC1_PERIPH->APB1RSTR },
    #endif

    #if defined( STM32_UART5_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_UART5RST, .reg = &RCC1_PERIPH->APB1RSTR },
    #endif
  };

  static const Chimera::Clock::Bus UART_SourceClock[ UART::NUM_UART_PERIPHS ] = {
    #if defined( STM32_UART4_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_UART5_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif
  };
  /* clang-format on */
#endif /* THOR_LLD_UART */

#if defined( THOR_USART )
  /* clang-format off */
  static const RegisterConfig USART_ClockConfig[ USART::NUM_USART_PERIPHS ]      = {
    #if defined( STM32_USART1_PERIPH_AVAILABLE )
    { .mask = APB2ENR_USART1EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif

    #if defined( STM32_USART2_PERIPH_AVAILABLE )
    { .mask = APB1ENR_USART2EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif

    #if defined( STM32_USART3_PERIPH_AVAILABLE )
    { .mask = APB1ENR_USART3EN, .reg  = &RCC1_PERIPH->APB1ENR },
    #endif

    #if defined( STM32_USART6_PERIPH_AVAILABLE )
    { .mask = APB2ENR_USART6EN, .reg  = &RCC1_PERIPH->APB2ENR },
    #endif
  };

  static const RegisterConfig USART_ResetConfig[ USART::NUM_USART_PERIPHS ]      = {
    #if defined( STM32_USART1_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_USART1RST, .reg = &RCC1_PERIPH->APB2RSTR },
    #endif

    #if defined( STM32_USART2_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_USART2RST, .reg = &RCC1_PERIPH->APB1RSTR },
    #endif

    #if defined( STM32_USART3_PERIPH_AVAILABLE )
    { .mask = APB1RSTR_USART3RST, .reg = &RCC1_PERIPH->APB1RSTR },
    #endif

    #if defined( STM32_USART6_PERIPH_AVAILABLE )
    { .mask = APB2RSTR_USART6RST, .reg = &RCC1_PERIPH->APB2RSTR },
    #endif
  };

  static const Chimera::Clock::Bus USART_SourceClock[ USART::NUM_USART_PERIPHS ] = {
    #if defined( STM32_USART1_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK2,
    #endif

    #if defined( STM32_USART2_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_USART3_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK1,
    #endif

    #if defined( STM32_USART6_PERIPH_AVAILABLE )
    Chimera::Clock::Bus::PCLK2,
    #endif
  };
  /* clang-format on */
#endif /* THOR_LLD_USART */

#if defined( THOR_USB )
  /* clang-format off */
  static const RegisterConfig USB_ClockConfig[ USB::NUM_USB_PERIPHS ]      = {
    { .mask = AHB2ENR_OTGFSEN,  .reg  = &RCC1_PERIPH->AHB2ENR },   /* Full Speed */
    { .mask = AHB1ENR_OTGHSEN,  .reg  = &RCC1_PERIPH->AHB1ENR },   /* High Speed */
  };

  static const RegisterConfig USB_ResetConfig[ USB::NUM_USB_PERIPHS ]      = {
    { .mask = AHB2RSTR_OTGFSRST, .reg = &RCC1_PERIPH->AHB2RSTR },  /* Full Speed */
    { .mask = AHB1RSTR_OTGHRST,  .reg = &RCC1_PERIPH->AHB1RSTR },  /* High Speed */
  };

  static const Chimera::Clock::Bus USB_SourceClock[ USB::NUM_USB_PERIPHS ] = {
    Chimera::Clock::Bus::AHB,
    Chimera::Clock::Bus::AHB
  };
  /* clang-format on */
#endif /* THOR_LLD_USB */

#if defined( THOR_WWDG )
  /* clang-format off */
  static const RegisterConfig WWDG_ClockConfig[ WWDG::NUM_WWDG_PERIPHS ]      = {
    { .mask = APB1ENR_WWDGEN, .reg  = &RCC1_PERIPH->APB1ENR }

  };

  static const RegisterConfig WWDG_ResetConfig[ WWDG::NUM_WWDG_PERIPHS ]      = {
    { .mask = 0, .reg = nullptr }
  };

  static const Chimera::Clock::Bus WWDG_SourceClock[ WWDG::NUM_WWDG_PERIPHS ] = {
    Chimera::Clock::Bus::APB1
  };
  /* clang-format on */
#endif /* THOR_LLD_WWDG */

  /*---------------------------------------------------------------------------
  Peripheral Control Structure Registration
  ---------------------------------------------------------------------------*/
  static const PCC sReg[] = {
  /* clang-format off */
    #if defined( THOR_ADC )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_ADC ),
      .elements         = ADC::NUM_ADC_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &ADC_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &ADC_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &ADC_SourceClock ),
      .getResourceIndex = ADC::getResourceIndex
    },
    #endif /* THOR_LLD_ADC */

    #if defined( THOR_CAN )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_CAN ),
      .elements         = CAN::NUM_CAN_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &CAN_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &CAN_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &CAN_SourceClock ),
      .getResourceIndex = CAN::getResourceIndex
    },
    #endif /* THOR_LLD_CAN */

    #if defined( THOR_CRS )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_CRS ),
      .elements         = CRS::NUM_CRS_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &CRS_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &CRS_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &CRS_SourceClock ),
      .getResourceIndex = nullptr
    },
    #endif /* THOR_LLD_CRS */

    #if defined( THOR_DMA )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_DMA ),
      .elements         = DMA::NUM_DMA_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &DMA_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &DMA_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &DMA_SourceClock ),
      .getResourceIndex = DMA::getResourceIndex
    },
    #endif /* THOR_LLD_DMA */

    #if defined( THOR_FLASH )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_FLASH ),
      .elements         = FLASH::NUM_FLASH_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &FLASH_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &FLASH_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &FLASH_SourceClock ),
      .getResourceIndex = FLASH::getResourceIndex
    },
    #endif /* THOR_LLD_FLASH */

    #if defined( THOR_GPIO )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_GPIO ),
      .elements         = GPIO::NUM_GPIO_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &GPIO_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &GPIO_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &GPIO_SourceClock ),
      .getResourceIndex = GPIO::getResourceIndex
    },
    #endif /* THOR_LLD_GPIO */

    #if defined( THOR_I2C )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_I2C ),
      .elements         = I2C::NUM_I2C_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &I2C_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &I2C_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &I2C_SourceClock ),
      .getResourceIndex = I2C::getResourceIndex
    },
    #endif /* THOR_LLD_I2C */

    #if defined( THOR_PWR )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_PWR ),
      .elements         = PWR::NUM_PWR_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &PWR_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &PWR_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &PWR_SourceClock ),
      .getResourceIndex = PWR::getResourceIndex
    },
    #endif /* THOR_LLD_PWR */

    #if defined( THOR_SDIO )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_SDIO ),
      .elements         = SDIO::NUM_SDIO_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &SDIO_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &SDIO_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &SDIO_SourceClock ),
      .getResourceIndex = SDIO::getResourceIndex
    },
    #endif /* THOR_LLD_SDIO */

    #if defined( THOR_SPI )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_SPI ),
      .elements         = SPI::NUM_SPI_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &SPI_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &SPI_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &SPI_SourceClock ),
      .getResourceIndex = SPI::getResourceIndex
    },
    #endif /* THOR_LLD_SPI */

    #if defined( THOR_SYSCFG )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_SYSCFG ),
      .elements         = SYS::NUM_SYSCFG_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &SYSCFG_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &SYSCFG_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &SYSCFG_SourceClock ),
      .getResourceIndex = SYS::getResourceIndex
    },
    #endif /* THOR_LLD_SYSCFG */

    #if defined( THOR_TIMER )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_TIMER ),
      .elements         = TIMER::NUM_TIMER_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &TIMER_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &TIMER_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &TIMER_SourceClock ),
      .getResourceIndex = TIMER::getGlobalResourceIndex
    },
    #endif /* THOR_LLD_TIMER */

    #if defined( THOR_USART )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_USART ),
      .elements         = USART::NUM_USART_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &USART_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &USART_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &USART_SourceClock ),
      .getResourceIndex = USART::getResourceIndex
    },
    #endif /* THOR_LLD_USART */

    #if defined( THOR_USB )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_USB ),
      .elements         = USB::NUM_USB_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &USB_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &USB_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &USB_SourceClock ),
      .getResourceIndex = USB::getResourceIndex
    },
    #endif /* THOR_LLD_USB */

    #if defined( THOR_WWDG )
    {
      .pType            = static_cast<uint8_t>( Chimera::Peripheral::Type::PERIPH_WWDG ),
      .elements         = WWDG::NUM_WWDG_PERIPHS,
      .bfControl        = 0,
      .reserved         = 0,
      .clock            = reinterpret_cast<const RegisterConfig*>( &WWDG_ClockConfig ),
      .clockLP          = nullptr,
      .reset            = reinterpret_cast<const RegisterConfig*>( &WWDG_ResetConfig ),
      .clockSource      = reinterpret_cast<const Chimera::Clock::Bus*>( &WWDG_SourceClock ),
      .getResourceIndex = Watchdog::getResourceIndex
    },
    #endif  /* THOR_LLD_WWDG */
  }; /* clang-format on */

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
#if defined( STM32_RCC1_PERIPH_AVAILABLE )
  RegisterMap   *RCC1_PERIPH = reinterpret_cast<RegisterMap *>( RCC1_BASE_ADDR );
  PCC LLD_CONST *PeripheralControlRegistry[ static_cast<size_t>( Chimera::Peripheral::Type::NUM_OPTIONS ) ];
#endif

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initializeRegistry()
  {
    using namespace Chimera::Peripheral;

    /*-------------------------------------------------------------------------
    First initialize the registry to empty
    -------------------------------------------------------------------------*/
    for ( size_t x = 0; x < ARRAY_COUNT( PeripheralControlRegistry ); x++ )
    {
      PeripheralControlRegistry[ x ] = nullptr;
    }

    /*-------------------------------------------------------------------------
    Fill in the defined data where possible
    -------------------------------------------------------------------------*/
    for ( size_t x = 0; x < ARRAY_COUNT( sReg ); x++ )
    {
      Type p = static_cast<Type>( sReg[ x ].pType );
      if ( p < Type::NUM_OPTIONS )
      {
        PeripheralControlRegistry[ sReg[ x ].pType ] = &sReg[ x ];
      }
    }
  }

}    // namespace Thor::LLD::RCC

#endif /* TARGET_STM32F4 && THOR_LLD_RCC */
