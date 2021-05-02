/********************************************************************************
 *  File Name:
 *    hw_dma_data.hpp
 *
 *  Description:
 *    STM32 Mappings for the DMA Peripheral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/dma>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/dma>


#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_DMA )

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
  RegisterMap *DMA1_PERIPH = reinterpret_cast<RegisterMap *>( DMA1_BASE_ADDR );
  StreamMap *DMA1_STREAM0    = reinterpret_cast<StreamMap *>( DMA1_STREAM0_BASE_ADDR );
  StreamMap *DMA1_STREAM1    = reinterpret_cast<StreamMap *>( DMA1_STREAM1_BASE_ADDR );
  StreamMap *DMA1_STREAM2    = reinterpret_cast<StreamMap *>( DMA1_STREAM2_BASE_ADDR );
  StreamMap *DMA1_STREAM3    = reinterpret_cast<StreamMap *>( DMA1_STREAM3_BASE_ADDR );
  StreamMap *DMA1_STREAM4    = reinterpret_cast<StreamMap *>( DMA1_STREAM4_BASE_ADDR );
  StreamMap *DMA1_STREAM5    = reinterpret_cast<StreamMap *>( DMA1_STREAM5_BASE_ADDR );
  StreamMap *DMA1_STREAM6    = reinterpret_cast<StreamMap *>( DMA1_STREAM6_BASE_ADDR );
  StreamMap *DMA1_STREAM7    = reinterpret_cast<StreamMap *>( DMA1_STREAM7_BASE_ADDR );

  RegisterMap *DMA2_PERIPH = reinterpret_cast<RegisterMap *>( DMA2_BASE_ADDR );
  StreamMap *DMA2_STREAM0    = reinterpret_cast<StreamMap *>( DMA2_STREAM0_BASE_ADDR );
  StreamMap *DMA2_STREAM1    = reinterpret_cast<StreamMap *>( DMA2_STREAM1_BASE_ADDR );
  StreamMap *DMA2_STREAM2    = reinterpret_cast<StreamMap *>( DMA2_STREAM2_BASE_ADDR );
  StreamMap *DMA2_STREAM3    = reinterpret_cast<StreamMap *>( DMA2_STREAM3_BASE_ADDR );
  StreamMap *DMA2_STREAM4    = reinterpret_cast<StreamMap *>( DMA2_STREAM4_BASE_ADDR );
  StreamMap *DMA2_STREAM5    = reinterpret_cast<StreamMap *>( DMA2_STREAM5_BASE_ADDR );
  StreamMap *DMA2_STREAM6    = reinterpret_cast<StreamMap *>( DMA2_STREAM6_BASE_ADDR );
  StreamMap *DMA2_STREAM7    = reinterpret_cast<StreamMap *>( DMA2_STREAM7_BASE_ADDR );

  /*-------------------------------------------------------------------------------
  Configuration Maps
  -------------------------------------------------------------------------------*/
  namespace Config
  {
    extern LLD_CONST uint32_t TransferMap[ EnumValue( Chimera::DMA::Direction::NUM_OPTIONS ) ] = {
      Configuration::Direction::P2M, Configuration::Direction::M2P, Configuration::Direction::M2M,
      Configuration::Direction::INVALID
    };

    extern LLD_CONST uint32_t ModeMap[ EnumValue( Chimera::DMA::Mode::NUM_OPTIONS ) ] = { Configuration::Mode::Normal,
                                                                                          Configuration::Mode::Circular,
                                                                                          Configuration::Mode::Periph };


    extern LLD_CONST uint32_t PriorityMap[ EnumValue( Chimera::DMA::Priority::NUM_OPTIONS ) ] = {
      Configuration::PriorityLevel::Low, Configuration::PriorityLevel::Medium, Configuration::PriorityLevel::High,
      Configuration::PriorityLevel::Ultra
    };


    // Can I turn this into an etl::unordered map?
    extern LLD_CONST StreamAttr RequestMap[ NUM_DMA_SOURCES ] = {
      /* clang-format off */
      /*-------------------------------------------------
      DMA 1 Stream 0
      -------------------------------------------------*/
      { Source::SPI3_RX,      ( ON_DMA1 | ON_STREAM_0 | ON_CHANNEL_0 ) },
      { Source::I2C1_RX,      ( ON_DMA1 | ON_STREAM_0 | ON_CHANNEL_1 ) },
      { Source::TIM4_CH1,     ( ON_DMA1 | ON_STREAM_0 | ON_CHANNEL_2 ) },
      { Source::UART5_RX,     ( ON_DMA1 | ON_STREAM_0 | ON_CHANNEL_4 ) },
      { Source::TIM5_CH3,     ( ON_DMA1 | ON_STREAM_0 | ON_CHANNEL_6 ) },
      { Source::TIM5_UP,      ( ON_DMA1 | ON_STREAM_0 | ON_CHANNEL_6 ) },

      /*-------------------------------------------------
      DMA 1 Stream 1
      -------------------------------------------------*/
      { Source::SPDIFRX_DT,   ( ON_DMA1 | ON_STREAM_1 | ON_CHANNEL_0 ) },
      { Source::I2C3_RX,      ( ON_DMA1 | ON_STREAM_1 | ON_CHANNEL_1 ) },
      { Source::TIM2_UP,      ( ON_DMA1 | ON_STREAM_1 | ON_CHANNEL_3 ) },
      { Source::TIM2_CH3,     ( ON_DMA1 | ON_STREAM_1 | ON_CHANNEL_3 ) },
      { Source::USART3_RX,    ( ON_DMA1 | ON_STREAM_1 | ON_CHANNEL_4 ) },
      { Source::TIM5_CH4,     ( ON_DMA1 | ON_STREAM_1 | ON_CHANNEL_6 ) },
      { Source::TIM5_TRIG,    ( ON_DMA1 | ON_STREAM_1 | ON_CHANNEL_6 ) },
      { Source::TIM6_UP,      ( ON_DMA1 | ON_STREAM_1 | ON_CHANNEL_7 ) },

      /*-------------------------------------------------
      DMA 1 Stream 2
      -------------------------------------------------*/
      { Source::SPI3_RX,      ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_0 ) },
      { Source::TIM7_UP,      ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_1 ) },
      { Source::FMPI2C1_RX,   ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_2 ) },
      { Source::I2C3_RX,      ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_3 ) },
      { Source::UART4_RX,     ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_4 ) },
      { Source::TIM3_CH4,     ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_5 ) },
      { Source::TIM3_UP,      ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_5 ) },
      { Source::TIM5_CH1,     ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_6 ) },
      { Source::I2C2_RX,      ( ON_DMA1 | ON_STREAM_2 | ON_CHANNEL_7 ) },

      /*-------------------------------------------------
      DMA 1 Stream 3
      -------------------------------------------------*/
      { Source::SPI2_RX,      ( ON_DMA1 | ON_STREAM_3 | ON_CHANNEL_0 ) },
      { Source::TIM4_CH2,     ( ON_DMA1 | ON_STREAM_3 | ON_CHANNEL_2 ) },
      { Source::USART3_TX,    ( ON_DMA1 | ON_STREAM_3 | ON_CHANNEL_4 ) },
      { Source::TIM5_CH4,     ( ON_DMA1 | ON_STREAM_3 | ON_CHANNEL_6 ) },
      { Source::TIM5_TRIG,    ( ON_DMA1 | ON_STREAM_3 | ON_CHANNEL_6 ) },
      { Source::I2C2_RX,      ( ON_DMA1 | ON_STREAM_3 | ON_CHANNEL_7 ) },


      /*-------------------------------------------------
      DMA 1 Stream 4
      -------------------------------------------------*/
      { Source::SPI2_TX,      ( ON_DMA1 | ON_STREAM_4 | ON_CHANNEL_0 ) },
      { Source::TIM7_UP,      ( ON_DMA1 | ON_STREAM_4 | ON_CHANNEL_1 ) },
      { Source::I2C3_TX,      ( ON_DMA1 | ON_STREAM_4 | ON_CHANNEL_3 ) },
      { Source::UART4_TX,     ( ON_DMA1 | ON_STREAM_4 | ON_CHANNEL_4 ) },
      { Source::TIM3_CH1,     ( ON_DMA1 | ON_STREAM_4 | ON_CHANNEL_5 ) },
      { Source::TIM3_TRIG,    ( ON_DMA1 | ON_STREAM_4 | ON_CHANNEL_5 ) },
      { Source::TIM5_CH2,     ( ON_DMA1 | ON_STREAM_4 | ON_CHANNEL_6 ) },
      { Source::USART3_TX,    ( ON_DMA1 | ON_STREAM_4 | ON_CHANNEL_7 ) },


      /*-------------------------------------------------
      DMA 1 Stream 5
      -------------------------------------------------*/
      { Source::SPI3_TX,      ( ON_DMA1 | ON_STREAM_5 | ON_CHANNEL_0 ) },
      { Source::I2C1_RX,      ( ON_DMA1 | ON_STREAM_5 | ON_CHANNEL_1 ) },
      { Source::FMPI2C1_RX,   ( ON_DMA1 | ON_STREAM_5 | ON_CHANNEL_2 ) },
      { Source::TIM2_CH1,     ( ON_DMA1 | ON_STREAM_5 | ON_CHANNEL_3 ) },
      { Source::USART2_RX,    ( ON_DMA1 | ON_STREAM_5 | ON_CHANNEL_4 ) },
      { Source::TIM3_CH2,     ( ON_DMA1 | ON_STREAM_5 | ON_CHANNEL_5 ) },
      { Source::DAC1,         ( ON_DMA1 | ON_STREAM_5 | ON_CHANNEL_7 ) },


      /*-------------------------------------------------
      DMA 1 Stream 6
      -------------------------------------------------*/
      { Source::SPDIFRX_CS,   ( ON_DMA1 | ON_STREAM_6 | ON_CHANNEL_0 ) },
      { Source::I2C1_TX,      ( ON_DMA1 | ON_STREAM_6 | ON_CHANNEL_1 ) },
      { Source::TIM4_UP,      ( ON_DMA1 | ON_STREAM_6 | ON_CHANNEL_2 ) },
      { Source::TIM2_CH2,     ( ON_DMA1 | ON_STREAM_6 | ON_CHANNEL_3 ) },
      { Source::TIM2_CH4,     ( ON_DMA1 | ON_STREAM_6 | ON_CHANNEL_3 ) },
      { Source::USART2_TX,    ( ON_DMA1 | ON_STREAM_6 | ON_CHANNEL_4 ) },
      { Source::TIM5_UP,      ( ON_DMA1 | ON_STREAM_6 | ON_CHANNEL_6 ) },
      { Source::DAC2,         ( ON_DMA1 | ON_STREAM_6 | ON_CHANNEL_7 ) },


      /*-------------------------------------------------
      DMA 1 Stream 7
      -------------------------------------------------*/
      { Source::SPI3_TX,      ( ON_DMA1 | ON_STREAM_7 | ON_CHANNEL_0 ) },
      { Source::I2C1_TX,      ( ON_DMA1 | ON_STREAM_7 | ON_CHANNEL_1 ) },
      { Source::TIM4_CH3,     ( ON_DMA1 | ON_STREAM_7 | ON_CHANNEL_2 ) },
      { Source::TIM2_UP,      ( ON_DMA1 | ON_STREAM_7 | ON_CHANNEL_3 ) },
      { Source::TIM2_CH4,     ( ON_DMA1 | ON_STREAM_7 | ON_CHANNEL_3 ) },
      { Source::UART5_TX,     ( ON_DMA1 | ON_STREAM_7 | ON_CHANNEL_4 ) },
      { Source::TIM3_CH3,     ( ON_DMA1 | ON_STREAM_7 | ON_CHANNEL_5 ) },
      { Source::I2C2_TX,      ( ON_DMA1 | ON_STREAM_7 | ON_CHANNEL_7 ) },


      /*-------------------------------------------------
      DMA 2 Stream 0
      -------------------------------------------------*/
      { Source::ADC1,         ( ON_DMA2 | ON_STREAM_0 | ON_CHANNEL_0 ) },
      { Source::ADC3,         ( ON_DMA2 | ON_STREAM_0 | ON_CHANNEL_2 ) },
      { Source::SPI1_RX,      ( ON_DMA2 | ON_STREAM_0 | ON_CHANNEL_3 ) },
      { Source::SPI4_RX,      ( ON_DMA2 | ON_STREAM_0 | ON_CHANNEL_4 ) },
      { Source::TIM1_TRIG,    ( ON_DMA2 | ON_STREAM_0 | ON_CHANNEL_6 ) },

      /*-------------------------------------------------
      DMA 2 Stream 1
      -------------------------------------------------*/
      { Source::SAI1_A,       ( ON_DMA2 | ON_STREAM_1 | ON_CHANNEL_0 ) },
      { Source::DCMI,         ( ON_DMA2 | ON_STREAM_1 | ON_CHANNEL_1 ) },
      { Source::ADC3,         ( ON_DMA2 | ON_STREAM_1 | ON_CHANNEL_2 ) },
      { Source::SPI4_TX,      ( ON_DMA2 | ON_STREAM_1 | ON_CHANNEL_4 ) },
      { Source::USART6_RX,    ( ON_DMA2 | ON_STREAM_1 | ON_CHANNEL_5 ) },
      { Source::TIM1_CH1,     ( ON_DMA2 | ON_STREAM_1 | ON_CHANNEL_6 ) },
      { Source::TIM8_UP,      ( ON_DMA2 | ON_STREAM_1 | ON_CHANNEL_7 ) },

      /*-------------------------------------------------
      DMA 2 Stream 2
      -------------------------------------------------*/
      { Source::TIM8_CH1,     ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_0 ) },
      { Source::TIM8_CH2,     ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_0 ) },
      { Source::TIM8_CH3,     ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_0 ) },
      { Source::ADC2,         ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_1 ) },
      { Source::SPI1_RX,      ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_3 ) },
      { Source::USART1_RX,    ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_4 ) },
      { Source::USART6_RX,    ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_5 ) },
      { Source::TIM1_CH2,     ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_6 ) },
      { Source::TIM8_CH1,     ( ON_DMA2 | ON_STREAM_2 | ON_CHANNEL_7 ) },

      /*-------------------------------------------------
      DMA 2 Stream 3
      -------------------------------------------------*/
      { Source::SAI1_A,       ( ON_DMA2 | ON_STREAM_3 | ON_CHANNEL_0 ) },
      { Source::ADC2,         ( ON_DMA2 | ON_STREAM_3 | ON_CHANNEL_1 ) },
      { Source::SPI1_TX,      ( ON_DMA2 | ON_STREAM_3 | ON_CHANNEL_3 ) },
      { Source::SDIO,         ( ON_DMA2 | ON_STREAM_3 | ON_CHANNEL_4 ) },
      { Source::SPI4_RX,      ( ON_DMA2 | ON_STREAM_3 | ON_CHANNEL_5 ) },
      { Source::TIM1_CH1,     ( ON_DMA2 | ON_STREAM_3 | ON_CHANNEL_6 ) },
      { Source::TIM8_CH2,     ( ON_DMA2 | ON_STREAM_3 | ON_CHANNEL_7 ) },


      /*-------------------------------------------------
      DMA 2 Stream 4
      -------------------------------------------------*/
      { Source::ADC1,         ( ON_DMA2 | ON_STREAM_4 | ON_CHANNEL_0 ) },
      { Source::SAI1_B,       ( ON_DMA2 | ON_STREAM_4 | ON_CHANNEL_1 ) },
      { Source::SAI2_A,       ( ON_DMA2 | ON_STREAM_4 | ON_CHANNEL_3 ) },
      { Source::SPI4_TX,      ( ON_DMA2 | ON_STREAM_4 | ON_CHANNEL_5 ) },
      { Source::TIM1_CH4,     ( ON_DMA2 | ON_STREAM_4 | ON_CHANNEL_6 ) },
      { Source::TIM1_TRIG,    ( ON_DMA2 | ON_STREAM_4 | ON_CHANNEL_6 ) },
      { Source::TIM1_COM,     ( ON_DMA2 | ON_STREAM_4 | ON_CHANNEL_6 ) },
      { Source::TIM8_CH3,     ( ON_DMA2 | ON_STREAM_4 | ON_CHANNEL_7 ) },


      /*-------------------------------------------------
      DMA 2 Stream 5
      -------------------------------------------------*/
      { Source::SAI1_B,       ( ON_DMA2 | ON_STREAM_5 | ON_CHANNEL_0 ) },
      { Source::SPI1_TX,      ( ON_DMA2 | ON_STREAM_5 | ON_CHANNEL_3 ) },
      { Source::USART1_RX,    ( ON_DMA2 | ON_STREAM_5 | ON_CHANNEL_4 ) },
      { Source::TIM1_UP,      ( ON_DMA2 | ON_STREAM_5 | ON_CHANNEL_6 ) },


      /*-------------------------------------------------
      DMA 2 Stream 6
      -------------------------------------------------*/
      { Source::TIM1_CH1,     ( ON_DMA2 | ON_STREAM_6 | ON_CHANNEL_0 ) },
      { Source::TIM1_CH2,     ( ON_DMA2 | ON_STREAM_6 | ON_CHANNEL_0 ) },
      { Source::TIM1_CH3,     ( ON_DMA2 | ON_STREAM_6 | ON_CHANNEL_0 ) },
      { Source::SAI2_B,       ( ON_DMA2 | ON_STREAM_6 | ON_CHANNEL_3 ) },
      { Source::SDIO,         ( ON_DMA2 | ON_STREAM_6 | ON_CHANNEL_4 ) },
      { Source::USART6_TX,    ( ON_DMA2 | ON_STREAM_6 | ON_CHANNEL_5 ) },
      { Source::TIM1_CH3,     ( ON_DMA2 | ON_STREAM_6 | ON_CHANNEL_6 ) },


      /*-------------------------------------------------
      DMA 2 Stream 7
      -------------------------------------------------*/
      { Source::SAI2_B,       ( ON_DMA2 | ON_STREAM_7 | ON_CHANNEL_0 ) },
      { Source::DCMI,         ( ON_DMA2 | ON_STREAM_7 | ON_CHANNEL_1 ) },
      { Source::QUADSPI,      ( ON_DMA2 | ON_STREAM_7 | ON_CHANNEL_3 ) },
      { Source::USART1_TX,    ( ON_DMA2 | ON_STREAM_7 | ON_CHANNEL_4 ) },
      { Source::USART6_TX,    ( ON_DMA2 | ON_STREAM_7 | ON_CHANNEL_5 ) },
      { Source::TIM8_CH4,     ( ON_DMA2 | ON_STREAM_7 | ON_CHANNEL_7 ) },
      { Source::TIM8_TRIG,    ( ON_DMA2 | ON_STREAM_7 | ON_CHANNEL_7 ) },
      { Source::TIM8_COM,     ( ON_DMA2 | ON_STREAM_7 | ON_CHANNEL_7 ) },

      /* clang-format on */
    };
  }    // namespace Config


  /*-------------------------------------------------------------------------------
  Resource Maps
  -------------------------------------------------------------------------------*/
  namespace Resource
  {
    extern LLD_CONST IRQn_Type IRQSignals[ NUM_DMA_STREAMS_TOTAL ] = {
      DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn,
      DMA1_Stream6_IRQn, DMA1_Stream7_IRQn, DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
      DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn
    };
  }

  /* clang-format on */
}    // namespace Thor::LLD::DMA
#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
