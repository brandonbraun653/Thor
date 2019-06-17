/********************************************************************************
 * File Name:
 *   spi_defaults.cpp
 *
 * Description:
 *   Provides definitions for external hardware configuration data
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/types/spi_types.hpp>

/* Thor Includes */
#include <Thor/defaults/spi_defaults.hpp>
#include <Thor/types/clock_types.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
namespace Thor::SPI
{
#if defined( SPI1 )
  static const SPIConfig spi1Config = {
#if ( defined( STM32F446xx ) || defined( STM32F767xx ) )
    // MOSI Pin
    { GPIOA, Thor::GPIO::PinNum::PIN_7, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI1 },

    // MISO Pin
    { GPIOA, Thor::GPIO::PinNum::PIN_6, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI1 },

    // SCK Pin
    { GPIOA, Thor::GPIO::PinNum::PIN_5, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI1 },

    // NSS Pin
    { GPIOA, Thor::GPIO::PinNum::PIN_4, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI1 },

    // SPI Instance Ptr
    SPI1,

    // IT_HW
    { SPI1_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_TX
    { DMA2_Stream5_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_RX
    { DMA2_Stream0_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA TX settings
    { DMA2_Stream5, DMA_CHANNEL_3, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA2_Stream0, DMA_CHANNEL_3, DMA_PERIPH_TO_MEMORY },

    // Clock Bus
    Thor::CLK::ClockBus::APB2_PERIPH
#endif
  };

  static const SPIConfig *spi1 = &spi1Config;
#else
  static const SPIConfig *spi1 = nullptr;
#endif

#if defined( SPI2 )
  static const SPIConfig spi2Config = {
#if defined( STM32F767xx )
    // MOSI Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_3, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI2 },

    // MISO Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_2, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI2 },

    // SCK Pin
    { GPIOB, Thor::GPIO::PinNum::PIN_10, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI2 },

    // NSS Pin
    { GPIOB, Thor::GPIO::PinNum::PIN_12, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI2 },
#endif

#if defined( STM32F446xx )
    // MOSI Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_1, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF7_SPI2 },    // Nucleo was B15 AF5

    // MISO Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_2, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI2 },    // Nucleo was B14 AF5

    // SCK Pin
    { GPIOB, Thor::GPIO::PinNum::PIN_10, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI2 },    // Nucleo was B3 AF5

    // NSS Pin
    { GPIOB, Thor::GPIO::PinNum::PIN_12, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI2 },
#endif

#if ( defined( STM32F446xx ) || defined( STM32F767xx ) )
    // SPI Instance Ptr
    SPI2,

    // IT_HW
    { SPI2_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_TX
    { DMA1_Stream4_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_RX
    { DMA1_Stream3_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA TX settings
    { DMA1_Stream4, DMA_CHANNEL_0, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA1_Stream3, DMA_CHANNEL_0, DMA_PERIPH_TO_MEMORY },

    // Clock Bus
    Thor::CLK::ClockBus::APB1_PERIPH
#endif 
  };

  static const SPIConfig *spi2 = &spi2Config;
#else
  static const SPIConfig *spi2 = nullptr;
#endif


#if defined( SPI3 )
  static const SPIConfig spi3Config = {
#if defined( STM32F767xx )
    // MOSI Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_12, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF6_SPI3 },

    // MISO Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_11, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF6_SPI3 },

    // SCK Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_10, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF6_SPI3 },

    // NSS Pin
    { GPIOA, Thor::GPIO::PinNum::PIN_15, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF6_SPI3 },
#endif

#if defined( STM32F446xx )
    // MOSI Pin
    { GPIOB, Thor::GPIO::PinNum::PIN_0, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF7_SPI3 },    // nucleo was B5 AF6

    // MISO Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_11, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF6_SPI3 },    // nucleo was B4

    // SCK Pin
    { GPIOC, Thor::GPIO::PinNum::PIN_10, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF6_SPI3 },    // nucleo was B3

    // NSS Pin
    { GPIOA, Thor::GPIO::PinNum::PIN_15, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF6_SPI3 },
#endif

#if defined( STM32F767xx ) || defined( STM32F446xx )
    // SPI Instance Ptr
    SPI3,

    // IT_HW
    { SPI3_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_TX
    { DMA1_Stream5_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_RX
    { DMA1_Stream2_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA TX settings
    { DMA1_Stream5, DMA_CHANNEL_0, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA1_Stream2, DMA_CHANNEL_0, DMA_PERIPH_TO_MEMORY },

    // Clock Bus
    Thor::CLK::ClockBus::APB1_PERIPH
#endif 
  };

  static const SPIConfig *spi3 = &spi3Config;
#else
  static const SPIConfig *spi3 = nullptr;
#endif


#if defined( SPI4 )
  static const SPIConfig spi4Config = {
#if defined( STM32F767xx ) || defined( STM32F446xx )
    // MOSI Pin
    { GPIOE, Thor::GPIO::PinNum::PIN_6, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI4 },

    // MISO Pin
    { GPIOE, Thor::GPIO::PinNum::PIN_5, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI4 },

    // SCK Pin
    { GPIOE, Thor::GPIO::PinNum::PIN_2, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI4 },

    // NSS Pin
    { GPIOE, Thor::GPIO::PinNum::PIN_4, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI4 },

    // SPI Instance Ptr
    SPI4,

    // IT_HW
    { SPI4_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_TX
    { DMA2_Stream1_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_RX
    { DMA2_Stream0_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA TX settings
    { DMA2_Stream1, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA2_Stream0, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY },

    // Clock Bus
    Thor::CLK::ClockBus::APB2_PERIPH
#endif 
  };

  static const SPIConfig *spi4 = &spi4Config;
#else
  static const SPIConfig *spi4 = nullptr;
#endif

#if defined( SPI5 )
  static const SPIConfig spi5Config = {
#if defined( STM32F767xx )
    // MOSI Pin
    { GPIOF, Thor::GPIO::PinNum::PIN_9, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI5 },

    // MISO Pin
    { GPIOF, Thor::GPIO::PinNum::PIN_8, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI5 },

    // SCK Pin
    { GPIOF, Thor::GPIO::PinNum::PIN_7, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI5 },

    // NSS Pin
    { GPIOF, Thor::GPIO::PinNum::PIN_6, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI5 },

    // SPI Instance Ptr
    SPI5,

    // IT_HW
    { SPI5_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_TX
    { DMA2_Stream4_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_RX
    { DMA2_Stream3_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA TX settings
    { DMA2_Stream4, DMA_CHANNEL_2, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA2_Stream3, DMA_CHANNEL_2, DMA_PERIPH_TO_MEMORY },

    // Clock Bus
    Thor::CLK::ClockBus::APB2_PERIPH
#endif 
  };

  static const SPIConfig *spi5 = &spi5Config;
#else
  static const SPIConfig *spi5 = nullptr;
#endif

#if defined( SPI6 )
  static const SPIConfig spi6Config = {
#if defined( STM32F767xx )
    // MOSI Pin
    { GPIOG, Thor::GPIO::PinNum::PIN_14, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI6 },

    // MISO Pin
    { GPIOG, Thor::GPIO::PinNum::PIN_12, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI6 },

    // SCK Pin
    { GPIOG, Thor::GPIO::PinNum::PIN_13, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI6 },

    // NSS Pin
    { GPIOG, Thor::GPIO::PinNum::PIN_8, Thor::GPIO::PinMode::ALT_PP, Thor::GPIO::PinSpeed::ULTRA_SPD,
      Thor::GPIO::PinPull::PULLUP, GPIO_AF5_SPI6 },

    // SPI Instance Ptr
    SPI6,

    // IT_HW
    { SPI6_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_TX
    { DMA2_Stream5_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA_IT_RX
    { DMA2_Stream6_IRQn, NVIC_PRIORITYGROUP_4, 0u },

    // DMA TX settings
    { DMA2_Stream5, DMA_CHANNEL_1, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA2_Stream6, DMA_CHANNEL_1, DMA_PERIPH_TO_MEMORY },

    // Clock Bus
    Thor::CLK::ClockBus::APB2_PERIPH
#endif 
  };

  static const SPIConfig *spi6 = &spi6Config;
#else
  static const SPIConfig *spi6 = nullptr;
#endif

  const std::array<const Thor::SPI::SPIConfig *const, Thor::SPI::MAX_SPI_CHANNELS + 1> hwConfig = {
    nullptr, spi1, spi2, spi3, spi4, spi5, spi6
  };

#if defined( STM32F7 ) || defined( STM32F4 )
  const DMA_InitTypeDef dflt_DMA_Init_TX = {
    DMA_CHANNEL_0,           /* Channel */
    DMA_MEMORY_TO_PERIPH,    /* Direction */
    DMA_PINC_DISABLE,        /* PeriphInc */
    DMA_MINC_ENABLE,         /* MemInc */
    DMA_PDATAALIGN_BYTE,     /* PeriphDataAlignment */
    DMA_MDATAALIGN_BYTE,     /* MemDataAlignment */
    DMA_NORMAL,              /* Mode */
    DMA_PRIORITY_LOW,        /* Priority */
    DMA_FIFOMODE_DISABLE,    /* FIFOMode */
    DMA_FIFO_THRESHOLD_FULL, /* FIFOThreshold */
    DMA_MBURST_SINGLE,       /* MemBurst */
    DMA_PBURST_SINGLE        /* PeriphBurst */
  };

  const DMA_InitTypeDef dflt_DMA_Init_RX = {
    DMA_CHANNEL_0,           /* Channel */
    DMA_PERIPH_TO_MEMORY,    /* Direction */
    DMA_PINC_DISABLE,        /* PeriphInc */
    DMA_MINC_ENABLE,         /* MemInc */
    DMA_PDATAALIGN_BYTE,     /* PeriphDataAlignment */
    DMA_MDATAALIGN_BYTE,     /* MemDataAlignment */
    DMA_NORMAL,              /* Mode */
    DMA_PRIORITY_LOW,        /* Priority */
    DMA_FIFOMODE_DISABLE,    /* FIFOMode */
    DMA_FIFO_THRESHOLD_FULL, /* FIFOThreshold */
    DMA_MBURST_SINGLE,       /* MemBurst */
    DMA_PBURST_SINGLE        /* PeriphBurst */
  };
#endif

#if defined( STM32F4 )
  const SPI_InitTypeDef dflt_SPI_Init = {
    SPI_MODE_MASTER,             /* Mode */
    SPI_DIRECTION_2LINES,        /* Direction */
    SPI_DATASIZE_8BIT,           /* DataSize */
    SPI_POLARITY_LOW,            /* CLKPolarity */
    SPI_PHASE_1EDGE,             /* CLKPhase */
    SPI_NSS_HARD_OUTPUT,         /* NSS */
    SPI_BAUDRATEPRESCALER_8,     /* BaudRatePrescaler */
    SPI_FIRSTBIT_MSB,            /* FirstBit */
    SPI_TIMODE_DISABLED,         /* TIMode */
    SPI_CRCCALCULATION_DISABLED, /* CRCCalculation */
    65535,                       /* CRCPolynomial */
  };
#endif

#if defined( STM32F7 )
  const SPI_InitTypeDef dflt_SPI_Init = {
    SPI_MODE_MASTER,             /* Mode */
    SPI_DIRECTION_2LINES,        /* Direction */
    SPI_DATASIZE_8BIT,           /* DataSize */
    SPI_POLARITY_LOW,            /* CLKPolarity */
    SPI_PHASE_1EDGE,             /* CLKPhase */
    SPI_NSS_HARD_OUTPUT,         /* NSS */
    SPI_BAUDRATEPRESCALER_8,     /* BaudRatePrescaler */
    SPI_FIRSTBIT_MSB,            /* FirstBit */
    SPI_TIMODE_DISABLE,          /* TIMode */
    SPI_CRCCALCULATION_DISABLE,  /* CRCCalculation */
    65535,                       /* CRCPolynomial */
    SPI_CRC_LENGTH_DATASIZE,     /* CRCLength */
    SPI_NSS_PULSE_ENABLE         /* NSSPMode */
  };
#endif


};    // namespace Thor::SPI

#endif 