/********************************************************************************
 * File Name:
 *     thor_defaults.cpp
 *
 * Description:
 *     Implements default peripheral configuration options
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/types/serial_types.hpp>

/* Thor Includes */
#include <Thor/definitions/interrupt_definitions.hpp>
#include <Thor/defaults/serial_defaults.hpp>

using namespace Thor::Interrupt;

namespace Thor::Serial
{
  /*------------------------------------------------
  Peripheral Configuration Defaults
  ------------------------------------------------*/
#if defined( USART1 )
#if defined( STM32F767xx ) || defined( STM32F446xx )
  static const SerialConfig usart1Config = {
    // Instance Ptr
    USART1,

    // IT_HW
    { USART1_IRQn, USART_IT_PREEMPT_PRIORITY, 0u },

    // DMA_IT_TX
    { DMA2_Stream7_IRQn, USART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA_IT_RX
    { DMA2_Stream2_IRQn, USART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA TX settings
    { DMA2_Stream7, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA2_Stream2, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY }
  };

  static const SerialConfig *serial1 = &usart1Config;
#endif
#else
  static const SerialConfig *serial1 = nullptr;
#endif

#if defined( USART2 )
#if defined( STM32F767xx ) || defined( STM32F446xx )
  static const SerialConfig usart2Config = {
    // Instance Ptr
    USART2,

    // IT_HW
    { USART2_IRQn, USART_IT_PREEMPT_PRIORITY, 0u },

    // DMA_IT_TX
    { DMA1_Stream6_IRQn, USART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA_IT_RX
    { DMA1_Stream5_IRQn, USART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA TX settings
    { DMA1_Stream6, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA1_Stream5, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY }
  };

  static const SerialConfig *serial2 = &usart2Config;
#endif
#else
  static const SerialConfig *serial2 = nullptr;
#endif


#if defined( USART3 )
#if defined( STM32F767xx ) || defined( STM32F446xx )
  static const SerialConfig usart3Config = {
    // Instance Ptr
    USART3,

    // IT_HW
    { USART3_IRQn, USART_IT_PREEMPT_PRIORITY, 0u },

    // DMA_IT_TX
    { DMA1_Stream3_IRQn, USART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA_IT_RX
    { DMA1_Stream1_IRQn, USART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA TX settings
    { DMA1_Stream3, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA1_Stream1, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY }
  };

  static const SerialConfig *serial3 = &usart3Config;
#endif
#else
  static const SerialConfig *serial3 = nullptr;
#endif


#if defined( UART4 )
#if defined( STM32F767xx ) || defined( STM32F446xx )
  static const SerialConfig uart4Config = {
    // Instance Ptr
    UART4,

    // IT_HW
    { UART4_IRQn, UART_IT_PREEMPT_PRIORITY, 0u },

    // DMA_IT_TX
    { DMA1_Stream4_IRQn, UART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA_IT_RX
    { DMA1_Stream2_IRQn, UART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA TX settings
    { DMA1_Stream4, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA1_Stream2, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY }
  };

  static const SerialConfig *serial4 = &uart4Config;
#endif
#else
  static const SerialConfig *serial4 = nullptr;
#endif


#if defined( UART5 )
#if defined( STM32F767xx ) || defined( STM32F446xx )
  static const SerialConfig uart5Config = {
    // Instance Ptr
    UART5,

    // IT_HW
    { UART5_IRQn, UART_IT_PREEMPT_PRIORITY, 0u },

    // DMA_IT_TX
    { DMA1_Stream7_IRQn, UART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA_IT_RX
    { DMA1_Stream0_IRQn, UART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA TX settings
    { DMA1_Stream7, DMA_CHANNEL_4, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA1_Stream0, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY }
  };

  static const SerialConfig *serial5 = &uart5Config;
#endif
#else
  static const SerialConfig *serial5 = nullptr;
#endif


#if defined( USART6 )
#if defined( STM32F767xx ) || defined( STM32F446xx )
  static const SerialConfig usart6Config = {
    // Instance Ptr
    USART6,

    // IT_HW
    { USART6_IRQn, USART_IT_PREEMPT_PRIORITY, 0u },

    // DMA_IT_TX
    { DMA2_Stream6_IRQn, USART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA_IT_RX
    { DMA2_Stream1_IRQn, USART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA TX settings
    { DMA2_Stream6, DMA_CHANNEL_5, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA2_Stream1, DMA_CHANNEL_5, DMA_PERIPH_TO_MEMORY }
  };

  static const SerialConfig *serial6 = &usart6Config;
#endif
#else
  static const SerialConfig *serial6 = nullptr;
#endif


#if defined( UART7 )
#if defined( STM32F767xx )
  static const SerialConfig uart7Config = {
    // Instance Ptr
    UART7,

    // IT_HW
    { UART7_IRQn, UART_IT_PREEMPT_PRIORITY, 0u },

    // DMA_IT_TX
    { DMA1_Stream1_IRQn, UART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA_IT_RX
    { DMA1_Stream3_IRQn, UART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA TX settings
    { DMA1_Stream1, DMA_CHANNEL_5, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA1_Stream3, DMA_CHANNEL_5, DMA_PERIPH_TO_MEMORY }
  };

  static const SerialConfig *serial7 = &uart7Config;
#endif
#else
  static const SerialConfig *serial7 = nullptr;
#endif


#if defined( UART8 )
#if defined( STM32F767xx )
  static const SerialConfig uart8Config = {
    // Instance Ptr
    UART8,

    // IT_HW
    { UART8_IRQn, UART_IT_PREEMPT_PRIORITY, 0u },

    // DMA_IT_TX
    { DMA1_Stream0_IRQn, UART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA_IT_RX
    { DMA1_Stream6_IRQn, UART_DMA_PREEMPT_PRIORITY, 0u },

    // DMA TX settings
    { DMA1_Stream0, DMA_CHANNEL_5, DMA_MEMORY_TO_PERIPH },

    // DMA RX settings
    { DMA1_Stream6, DMA_CHANNEL_5, DMA_PERIPH_TO_MEMORY }
  };

  static const SerialConfig *serial8 = &uart8Config;
#endif
#else
  static const SerialConfig *serial8 = nullptr;
#endif


  const std::array<const SerialConfig *const, Thor::Serial::MAX_SERIAL_CHANNELS + 1> hwConfig = { nullptr, serial1, serial2,
                                                                                                  serial3, serial4, serial5,
                                                                                                  serial6, serial7, serial8 };


#if defined( STM32F7 ) || defined( STM32F4 )
  const DMA_InitTypeDef dflt_DMA_Init_TX = {
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
  const USART_InitTypeDef dflt_USART_Init = {
    static_cast<uint32_t>( Chimera::Serial::BaudRate::SERIAL_BAUD_115200 ), /* BaudRate */
    USART_WORDLENGTH_8B,                                                    /* WordLength */
    USART_STOPBITS_1,                                                       /* StopBits */
    USART_PARITY_NONE,                                                      /* Parity */
    USART_MODE_TX_RX,                                                       /* Mode */
    USART_POLARITY_LOW,                                                     /* CLKPolarity */
    USART_PHASE_1EDGE,                                                      /* CLKPhase */
    USART_LASTBIT_DISABLE                                                   /* CLKLastBit */
  };

  const UART_InitTypeDef dflt_UART_Init = {
    static_cast<uint32_t>( Chimera::Serial::BaudRate::SERIAL_BAUD_115200 ), /* BaudRate */
    UART_WORDLENGTH_8B,                                                     /* WordLength */
    UART_STOPBITS_1,                                                        /* StopBits */
    UART_PARITY_NONE,                                                       /* Parity */
    UART_MODE_TX_RX,                                                        /* Mode */
    UART_HWCONTROL_NONE,                                                    /* HwFlowCtl */
    UART_OVERSAMPLING_16,                                                   /* OverSampling */
  };
#endif

#if defined( STM32F7 )
  const USART_InitTypeDef dflt_USART_Init = {
    static_cast<uint32_t>( BaudRate::SERIAL_BAUD_115200 ), /* BaudRate */
    USART_WORDLENGTH_8B,                                   /* WordLength */
    USART_STOPBITS_1,                                      /* StopBits */
    USART_PARITY_NONE,                                     /* Parity */
    USART_MODE_TX_RX,                                      /* Mode */
    USART_OVERSAMPLING_16,                                 /* OverSampling */
    USART_POLARITY_LOW,                                    /* CLKPolarity */
    USART_PHASE_1EDGE,                                     /* CLKPhase */
    USART_LASTBIT_DISABLE                                  /* CLKLastBit */
  };

  const UART_InitTypeDef dflt_UART_Init = {
    static_cast<uint32_t>( BaudRate::SERIAL_BAUD_115200 ), /* BaudRate */
    UART_WORDLENGTH_8B,                                    /* WordLength */
    UART_STOPBITS_1,                                       /* StopBits */
    UART_PARITY_NONE,                                      /* Parity */
    UART_MODE_TX_RX,                                       /* Mode */
    UART_HWCONTROL_NONE,                                   /* HwFlowCtl */
    UART_OVERSAMPLING_16,                                  /* OverSampling */
    UART_ONE_BIT_SAMPLE_DISABLE                            /* OneBitSampling */
  };

  const UART_AdvFeatureInitTypeDef dflt_UART_AdvInit = {
    UART_ADVFEATURE_NO_INIT,                 /* AdvFeatureInit */
    UART_ADVFEATURE_TXINV_DISABLE,           /* TxPinLevelInvert */
    UART_ADVFEATURE_RXINV_DISABLE,           /* RxPinLevelInvert */
    UART_ADVFEATURE_DATAINV_DISABLE,         /* DataInvert */
    UART_ADVFEATURE_SWAP_DISABLE,            /* Swap */
    UART_ADVFEATURE_OVERRUN_DISABLE,         /* OverrunDisable */
    UART_ADVFEATURE_DMA_DISABLEONRXERROR,    /* DMADisableonRxError */
    UART_ADVFEATURE_AUTOBAUDRATE_DISABLE,    /* AutoBaudRateEnable */
    UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT, /* AutoBaudRateMode	 */
    UART_ADVFEATURE_MSBFIRST_DISABLE         /* MSBFirst */
  };
#endif
}    // namespace Thor::Defaults::Serial
