/********************************************************************************
 *  File Name:
 *    hld_prv_dma_resources.cpp
 *
 *  Description:
 *    Thor resources used in the DMA driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/dma>
#include <Thor/hld/dma/hld_prv_dma_resources.hpp>
#include <Thor/lld/interface/dma/dma.hpp>
#include <Thor/lld/interface/dma/dma_types.hpp>

namespace Thor::DMA
{
  using namespace ::Thor::LLD::DMA;
  using namespace ::Thor::LLD::DMA::ConfigBitFields;

  /* clang-format off */
  #if defined( TARGET_STM32F4 ) || defined( TARGET_STM32F7 )
  /*------------------------------------------------
  Taken from tables 28 & 29 on RM0390 datasheet
  ------------------------------------------------*/
  std::array<Thor::LLD::DMA::StreamResources, NUM_REQUEST_GENERATORS> RequestGenerators = {{
    { Source::NONE,           ConfigBitFields::EMPTY_CONFIG,                    {} },
    /* USART */
    { Source::S_USART1_RX,    DMA_ON_DMA2 | DMA_STREAM_2 | DMA_CHANNEL_4,       {} },
    { Source::S_USART1_TX,    DMA_ON_DMA2 | DMA_STREAM_7 | DMA_CHANNEL_4,       {} },
    { Source::S_USART2_RX,    DMA_ON_DMA1 | DMA_STREAM_5 | DMA_CHANNEL_4,       {} },
    { Source::S_USART2_TX,    DMA_ON_DMA1 | DMA_STREAM_6 | DMA_CHANNEL_4,       {} },
    { Source::S_USART3_RX,    DMA_ON_DMA1 | DMA_STREAM_1 | DMA_CHANNEL_4,       {} },
    { Source::S_USART3_TX,    DMA_ON_DMA1 | DMA_STREAM_3 | DMA_CHANNEL_4,       {} },
    { Source::S_USART6_RX,    DMA_ON_DMA2 | DMA_STREAM_1 | DMA_CHANNEL_5,       {} },
    { Source::S_USART6_TX,    DMA_ON_DMA2 | DMA_STREAM_6 | DMA_CHANNEL_5,       {} },
  }};


//  const std::array<std::array<uint8_t, 8>, 8> dma1RequestMapping = {{
//    { Source::S_SPI3_RX,    Source::S_SPDIFRX_DT,   Source::S_SPI3_RX,      Source::S_SPI2_RX,    Source::S_SPI2_TX,      Source::S_SPI3_TX,      Source::S_SPDIFRX_CS,   Source::S_SPI3_TX   },
//    { Source::S_I2C1_RX,    Source::S_I2C3_RX,      Source::S_TIM7_UP,      Source::NONE,         Source::S_TIM7_UP,      Source::S_I2C1_RX,      Source::S_I2C1_TX,      Source::S_I2C1_TX   },
//    { Source::S_TIM4_CH1,   Source::NONE,           Source::S_FMPI2C1_RX,   Source::S_TIM4_CH2,   Source::NONE,           Source::S_FMPI2C1_RX,   Source::S_TIM4_UP,      Source::S_TIM4_CH3  },
//    { Source::NONE,         Source::S_TIM2_CH3,     Source::S_I2C3_RX,      Source::NONE,         Source::S_I2C3_TX,      Source::S_TIM2_CH1,     Source::S_TIM2_CH2,     Source::S_TIM2_CH4  },
//    { Source::S_UART5_RX,   Source::S_USART3_RX,    Source::S_UART4_RX,     Source::S_USART3_TX,  Source::S_UART4_TX,     Source::S_USART2_RX,    Source::S_USART2_TX,    Source::S_UART5_TX  },
//    { Source::NONE,         Source::NONE,           Source::S_TIM3_CH4,     Source::NONE,         Source::S_TIM3_CH1,     Source::S_TIM3_CH2,     Source::NONE,           Source::S_TIM3_CH3  },
//    { Source::S_TIM5_CH3,   Source::S_TIM5_CH4,     Source::S_TIM5_CH1,     Source::S_TIM5_TRIG,  Source::S_TIM5_CH2,     Source::NONE,           Source::S_TIM5_UP,      Source::NONE        },
//    { Source::NONE,         Source::S_TIM6_UP,      Source::S_I2C2_RX,      Source::S_I2C2_RX,    Source::S_USART3_TX,    Source::S_DAC1,         Source::S_DAC2,         Source::S_I2C2_TX   }
//  }};

//  const std::array<std::array<uint8_t, 8>, 8> dma2RequestMapping = {{
//    { Source::S_ADC1,       Source::S_SAI1_A,       Source::S_TIM8_CH1,     Source::S_SAI1_A,     Source::S_ADC1,         Source::S_SAI1_B,       Source::S_TIM1_CH1,     Source::S_SAI2_B    },
//    { Source::NONE,         Source::S_DCMI,         Source::S_ADC2,         Source::S_ADC2,       Source::S_SAI1_B,       Source::NONE,           Source::NONE,           Source::S_DCMI      },
//    { Source::S_ADC3,       Source::S_ADC3,         Source::NONE,           Source::NONE,         Source::NONE,           Source::NONE,           Source::NONE,           Source::NONE        },
//    { Source::S_SPI1_RX,    Source::NONE,           Source::S_SPI1_RX,      Source::S_SPI1_TX,    Source::S_SAI2_A,       Source::S_SPI1_TX,      Source::S_SAI2_B,       Source::S_QUADSPI   },
//    { Source::S_SPI4_RX,    Source::S_SPI4_TX,      Source::S_USART1_RX,    Source::S_SDIO,       Source::NONE,           Source::S_USART1_RX,    Source::S_SDIO,         Source::S_USART1_TX },
//    { Source::NONE,         Source::S_USART6_RX,    Source::S_USART6_RX,    Source::S_SPI4_RX,    Source::S_SPI4_TX,      Source::NONE,           Source::S_USART6_TX,    Source::S_USART6_TX },
//    { Source::S_TIM1_TRIG,  Source::S_TIM1_CH1,     Source::S_TIM1_CH2,     Source::S_TIM1_CH1,   Source::S_TIM1_CH4,     Source::S_TIM1_UP,      Source::S_TIM1_CH3,     Source::NONE        },
//    { Source::NONE,         Source::S_TIM8_UP,      Source::S_TIM8_CH1,     Source::S_TIM8_CH2,   Source::S_TIM8_CH3,     Source::NONE,           Source::NONE,           Source::S_TIM8_CH4  }
//  }};
  #endif
/* clang-format on */
}