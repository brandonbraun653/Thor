#pragma once
#ifndef THOR_DEFINITIONS_H_
#define THOR_DEFINITIONS_H_

/* C/C++ Includes */
#include <limits>
#include <cstdint>

/* Thor Includes */
#include <Thor/headers.hpp>

/* FreeRTOS Includes */
#if defined( USING_FREERTOS )
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif

#ifndef UNUSED
#define UNUSED( x ) ( void )x
#endif

namespace Thor
{
  enum class ClockBus : uint8_t
  {
    APB1_PERIPH,
    APB2_PERIPH,
    APB1_TIMER,
    APB2_TIMER
  };

  static inline constexpr Chimera::Status_t convertHALStatus(const HAL_StatusTypeDef status)
  {
    switch (status)
    {
    case HAL_ERROR:
      return Chimera::CommonStatusCodes::FAIL;
      break;

    case HAL_BUSY:
      return Chimera::CommonStatusCodes::BUSY;
      break;

    case HAL_OK:
      return Chimera::CommonStatusCodes::OK;
      break;

    case HAL_TIMEOUT:
      return Chimera::CommonStatusCodes::TIMEOUT;
      break;

    default:
      return Chimera::CommonStatusCodes::UNKNOWN_ERROR;
      break;
    }
  }

  namespace Interrupt
  {
#if defined( USING_FREERTOS )
    const uint32_t EXTI0_MAX_IRQn_PRIORITY   = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
    const uint32_t MAX_PENDING_TASK_TRIGGERS = 10; /**< The largest number of queued events at any given time */

    /** The various types of triggers that can be used to unlock a FreeRTOS thread */
    enum Trigger : uint8_t
    {
      RX_COMPLETE,
      TX_COMPLETE,
      TXRX_COMPLETE,
      TRANSFER_ERROR,
      BUFFERED_TX_COMPLETE,
      BUFFERED_TXRX_COMPLETE,
      MAX_SOURCES
    };
#endif
  }    // namespace Interrupt

  namespace GPIO
  {
    typedef GPIO_TypeDef *PinPort;

    constexpr uint32_t NOALTERNATE = ( 0x08000CC8 );    // Default value for the alternate configuration var

    enum class PinNum : uint32_t
    {
      PIN_0   = GPIO_PIN_0,
      PIN_1   = GPIO_PIN_1,
      PIN_2   = GPIO_PIN_2,
      PIN_3   = GPIO_PIN_3,
      PIN_4   = GPIO_PIN_4,
      PIN_5   = GPIO_PIN_5,
      PIN_6   = GPIO_PIN_6,
      PIN_7   = GPIO_PIN_7,
      PIN_8   = GPIO_PIN_8,
      PIN_9   = GPIO_PIN_9,
      PIN_10  = GPIO_PIN_10,
      PIN_11  = GPIO_PIN_11,
      PIN_12  = GPIO_PIN_12,
      PIN_13  = GPIO_PIN_13,
      PIN_14  = GPIO_PIN_14,
      PIN_15  = GPIO_PIN_15,
      PIN_ALL = GPIO_PIN_All,

      MAX_PINS  = 16,
      NOT_A_PIN = std::numeric_limits<std::int32_t>::max()
    };

    enum class PinMode : uint32_t
    {
      INPUT              = GPIO_MODE_INPUT,
      OUTPUT_PP          = GPIO_MODE_OUTPUT_PP,
      OUTPUT_OD          = GPIO_MODE_OUTPUT_OD,
      ALT_PP             = GPIO_MODE_AF_PP,
      ALT_OD             = GPIO_MODE_AF_OD,
      ANALOG             = GPIO_MODE_ANALOG,
      IT_RISING          = GPIO_MODE_IT_RISING,
      IT_FALLING         = GPIO_MODE_IT_FALLING,
      IT_RISING_FALLING  = GPIO_MODE_IT_RISING_FALLING,
      EVT_RISING         = GPIO_MODE_EVT_RISING,
      EVT_FALLING        = GPIO_MODE_EVT_FALLING,
      EVT_RISING_FALLING = GPIO_MODE_EVT_RISING_FALLING,

      NUM_MODES,
      UNKNOWN_MODE
    };

    enum class PinSpeed : uint32_t
    {
      LOW_SPD    = GPIO_SPEED_FREQ_LOW,
      MEDIUM_SPD = GPIO_SPEED_FREQ_MEDIUM,
      HIGH_SPD   = GPIO_SPEED_FREQ_HIGH,
      ULTRA_SPD  = GPIO_SPEED_FREQ_VERY_HIGH,

      NUM_SPEEDS,
      UNKNOWN_SPEED
    };

    enum class PinPull : uint32_t
    {
      NOPULL = GPIO_NOPULL,
      PULLUP = GPIO_PULLUP,
      PULLDN = GPIO_PULLDOWN,

      NUM_PULL,
      UNKNOWN_PULL
    };

    struct PinConfig
    {
      PinPort GPIOx      = GPIOA;
      PinSpeed speed     = PinSpeed::MEDIUM_SPD;
      PinMode mode       = PinMode::INPUT;
      PinNum pinNum      = PinNum::NOT_A_PIN;
      PinPull pull       = PinPull::NOPULL;
      uint32_t alternate = NOALTERNATE;
    };
  }    // namespace GPIO

  namespace TIMER
  {
    const unsigned int MAX_CHANNELS     = 16;
    const unsigned int MAX_SUB_CHANNELS = 6;
    const unsigned int MAX_ALT_PORTS    = 4;
    const unsigned int MAX_ALT_PINS     = 4;

    /*--------------------------
     * Hardware Descriptors
     *--------------------------*/
    const uint32_t timerBaseAddresses[] = {
#if defined( STM32F446xx ) || defined( STM32F767xx )
      0,
      /* Indexing offset since no TIM0 */
      TIM1_BASE, TIM2_BASE, TIM3_BASE, TIM4_BASE, TIM5_BASE, TIM6_BASE, TIM7_BASE, TIM8_BASE, TIM9_BASE, TIM10_BASE, TIM11_BASE,
      TIM12_BASE, TIM13_BASE, TIM14_BASE
#endif
    };

    enum TimerCategory
    {
      TIMER_BASIC,
      TIMER_GENERAL_PURPOSE,
      TIMER_ADVANCED,
      TIMER_LOW_POWER
    };

    enum TimerChannelSize
    {
      TIMER_BASIC_CHANNELS     = 2u,
      TIMER_BASIC_SUB_CHANNELS = 1u,

      TIMER_GENERAL_PURPOSE_CHANNELS     = 10u,
      TIMER_GENERAL_PURPOSE_SUB_CHANNELS = 4u,

      TIMER_ADVANCED_CHANNELS     = 2u,
      TIMER_ADVANCED_SUB_CHANNELS = 6u,

      TIMER_LOW_POWER_CHANNELS     = 1u,
      TIMER_LOW_POWER_SUB_CHANNELS = 1u
    };

    enum TimerSize
    {
      TIMER_16BIT = 1u,
      TIMER_32BIT = 2u
    };

    enum TimerDirection
    {
      TIMER_UP            = 1u,
      TIMER_DOWN          = 2u,
      TIMER_AUTO_RELOAD   = 4u,
      TIMER_DIRECTION_ALL = ( TIMER_UP | TIMER_DOWN | TIMER_AUTO_RELOAD )
    };

    enum TimerModes
    {
      TIMER_INPUT_CAPTURE  = 1u,
      TIMER_OUTPUT_COMPARE = 2u,
      TIMER_PWM            = 4u,
      TIMER_ONE_PULSE      = 8u,
      TIMER_ENCODER        = 16u,
      TIMER_BASE           = 32u,
      TIMER_MODE_TIER_1 =
          ( TIMER_INPUT_CAPTURE | TIMER_OUTPUT_COMPARE | TIMER_PWM | TIMER_ONE_PULSE | TIMER_BASE | TIMER_ENCODER ),
      TIMER_MODE_TIER_2 = ( TIMER_INPUT_CAPTURE | TIMER_OUTPUT_COMPARE | TIMER_PWM | TIMER_ONE_PULSE | TIMER_BASE ),
      TIMER_MODE_TIER_3 = ( TIMER_PWM | TIMER_ONE_PULSE | TIMER_BASE | TIMER_ENCODER )
    };

    enum TimerClockSource
    {
      ON_APB1,
      ON_APB2
    };

    /*--------------------------
     * Functionality Descriptors
     *--------------------------*/
    enum OCModes
    {
      OC_TIMING,
      OC_ACTIVE,
      OC_INACTIVE,
      OC_TOGGLE,
      OC_PWM1,
      OC_PWM2,
      OC_FORCED_ACTIVE,
      OC_FORCED_INACTIVE,
      OC_RETRIG_OPM1,
      OC_RETRIG_OPM2,
      OC_COMBINED_PWM1,
      OC_COMBINED_PWM2,
      OC_ASSYM_PWM1,
      OC_ASSYM_PWM2
    };

    enum OCPolarity
    {
      /* Normal Output Polarity */
      OC_HIGH,
      OC_LOW,

      /* Complementary Output Polarity*/
      OC_NHIGH,
      OC_NLOW
    };

    enum OCIdleState
    {
      /* Normal Idle State */
      OC_SET,
      OC_RESET,

      /* Complementary Idle State*/
      OC_NSET,
      OC_NRESET
    };
  }    // namespace TIMER

  namespace SPI
  {
    constexpr uint8_t MAX_SPI_CHANNELS     = 6;
    constexpr uint8_t SPI_BUFFER_SIZE      = 32;
    constexpr uint32_t BLOCKING_TIMEOUT_MS = 100;

    enum class ChipSelectMode : uint8_t
    {
      MANUAL,                /**< Manually control the state of the chip select line */
      AUTO_BETWEEN_TRANSFER, /**< Automatically twiddle the chip select between transfers */
      AUTO_AFTER_TRANSFER,   /**< Automatically disable the chip select after all transfers complete */

      NUM_CS_MODES,
      UNKNOWN_CS_MODE
    };
  }    // namespace SPI

  /** @namespace Thor::DMA */
  namespace DMA
  {
/* Useful Macros for Generating DMA Register Addresses*/
#define DMA_OFFSET_LISR 0x00U
#define DMA_OFFSET_HISR 0x04U
#define DMA_OFFSET_LIFCR 0x08U
#define DMA_OFFSET_HIFCR 0x0CU
#define DMA_OFFSET_SxCR( STREAM_NUMBER ) ( 0x10U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxNDTR( STREAM_NUMBER ) ( 0x14U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxPAR( STREAM_NUMBER ) ( 0x18U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxM0AR( STREAM_NUMBER ) ( 0x1CU + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxM1AR( STREAM_NUMBER ) ( 0x20U + 0x18U * ( uint32_t )STREAM_NUMBER )

// The data sheet has conflicting address definitions for this register. The register map
// does not match manual calculations using the equation below...unsure which is right
#define DMA_OFFSET_SxFCR( STREAM_NUMBER ) ( 0x24U + 0x24U * ( uint32_t )STREAM_NUMBER )

/* Register Definitions for DMA1 */
#define DMA1_LISR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_LISR ) )
#define DMA1_HISR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_HISR ) )
#define DMA1_LIFCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_LIFCR ) )
#define DMA1_HIFCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_HIFCR ) )
#define DMA1_S0CR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 0 ) ) )
#define DMA1_S1CR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 1 ) ) ) /* DMA1_SxCR */
#define DMA1_S2CR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 2 ) ) )
#define DMA1_S3CR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 3 ) ) )
#define DMA1_S4CR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 4 ) ) )
#define DMA1_S5CR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 5 ) ) )
#define DMA1_S6CR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 6 ) ) )
#define DMA1_S7CR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 7 ) ) )
#define DMA1_S0NDTR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 0 ) ) )
#define DMA1_S1NDTR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 1 ) ) ) /* DMA1_SxNDTR */
#define DMA1_S2NDTR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 2 ) ) )
#define DMA1_S3NDTR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 3 ) ) )
#define DMA1_S4NDTR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 4 ) ) )
#define DMA1_S5NDTR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 5 ) ) )
#define DMA1_S6NDTR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 6 ) ) )
#define DMA1_S7NDTR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 7 ) ) )
#define DMA1_S0PAR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 0 ) ) )
#define DMA1_S1PAR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 1 ) ) ) /* DMA1_SxPAR */
#define DMA1_S2PAR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 2 ) ) )
#define DMA1_S3PAR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 3 ) ) )
#define DMA1_S4PAR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 4 ) ) )
#define DMA1_S5PAR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 5 ) ) )
#define DMA1_S6PAR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 6 ) ) )
#define DMA1_S7PAR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 7 ) ) )
#define DMA1_S0M0AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 0 ) ) )
#define DMA1_S1M0AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 1 ) ) ) /* DMA1_SxM0AR */
#define DMA1_S2M0AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 2 ) ) )
#define DMA1_S3M0AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 3 ) ) )
#define DMA1_S4M0AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 4 ) ) )
#define DMA1_S5M0AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 5 ) ) )
#define DMA1_S6M0AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 6 ) ) )
#define DMA1_S7M0AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 7 ) ) )
#define DMA1_S0M1AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 0 ) ) )
#define DMA1_S1M1AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 1 ) ) ) /* DMA1_SxM1AR*/
#define DMA1_S2M1AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 2 ) ) )
#define DMA1_S3M1AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 3 ) ) )
#define DMA1_S4M1AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 4 ) ) )
#define DMA1_S5M1AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 5 ) ) )
#define DMA1_S6M1AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 6 ) ) )
#define DMA1_S7M1AR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 7 ) ) )
#define DMA1_S0FCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 0 ) ) )
#define DMA1_S1FCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 1 ) ) ) /* DMA1_SxFCR */
#define DMA1_S2FCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 2 ) ) )
#define DMA1_S3FCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 3 ) ) )
#define DMA1_S4FCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 4 ) ) )
#define DMA1_S5FCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 5 ) ) )
#define DMA1_S6FCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 6 ) ) )
#define DMA1_S7FCR ( *( uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 7 ) ) )

/* Register Definitions for DMA2 */
#define DMA2_LISR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_LISR ) )
#define DMA2_HISR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_HISR ) )
#define DMA2_LIFCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_LIFCR ) )
#define DMA2_HIFCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_HIFCR ) )
#define DMA2_S0CR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 0 ) ) )
#define DMA2_S1CR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 1 ) ) ) /* DMA2_SxCR */
#define DMA2_S2CR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 2 ) ) )
#define DMA2_S3CR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 3 ) ) )
#define DMA2_S4CR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 4 ) ) )
#define DMA2_S5CR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 5 ) ) )
#define DMA2_S6CR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 6 ) ) )
#define DMA2_S7CR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 7 ) ) )
#define DMA2_S0NDTR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 0 ) ) )
#define DMA2_S1NDTR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 1 ) ) ) /* DMA2_SxNDTR */
#define DMA2_S2NDTR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 2 ) ) )
#define DMA2_S3NDTR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 3 ) ) )
#define DMA2_S4NDTR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 4 ) ) )
#define DMA2_S5NDTR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 5 ) ) )
#define DMA2_S6NDTR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 6 ) ) )
#define DMA2_S7NDTR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 7 ) ) )
#define DMA2_S0PAR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 0 ) ) )
#define DMA2_S1PAR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 1 ) ) ) /* DMA2_SxPAR */
#define DMA2_S2PAR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 2 ) ) )
#define DMA2_S3PAR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 3 ) ) )
#define DMA2_S4PAR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 4 ) ) )
#define DMA2_S5PAR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 5 ) ) )
#define DMA2_S6PAR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 6 ) ) )
#define DMA2_S7PAR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 7 ) ) )
#define DMA2_S0M0AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 0 ) ) )
#define DMA2_S1M0AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 1 ) ) ) /* DMA2_SxM0AR */
#define DMA2_S2M0AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 2 ) ) )
#define DMA2_S3M0AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 3 ) ) )
#define DMA2_S4M0AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 4 ) ) )
#define DMA2_S5M0AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 5 ) ) )
#define DMA2_S6M0AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 6 ) ) )
#define DMA2_S7M0AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 7 ) ) )
#define DMA2_S0M1AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 0 ) ) )
#define DMA2_S1M1AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 1 ) ) ) /* DMA2_SxM1AR*/
#define DMA2_S2M1AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 2 ) ) )
#define DMA2_S3M1AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 3 ) ) )
#define DMA2_S4M1AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 4 ) ) )
#define DMA2_S5M1AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 5 ) ) )
#define DMA2_S6M1AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 6 ) ) )
#define DMA2_S7M1AR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 7 ) ) )
#define DMA2_S0FCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 0 ) ) )
#define DMA2_S1FCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 1 ) ) ) /* DMA2_SxFCR */
#define DMA2_S2FCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 2 ) ) )
#define DMA2_S3FCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 3 ) ) )
#define DMA2_S4FCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 4 ) ) )
#define DMA2_S5FCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 5 ) ) )
#define DMA2_S6FCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 6 ) ) )
#define DMA2_S7FCR ( *( uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 7 ) ) )

    enum TransferDirection
    {
      PERIPH_TO_MEM,
      MEM_TO_PERIPH,
      MEM_TO_MEM,
      TRANSFER_DIRECTION_UNDEFINED
    };
  }    // namespace DMA

  /** @namespace Thor::UART */
  namespace UART
  {
    constexpr uint8_t MAX_UART_CHANNELS = 4;  /**< Total possible UART specific channels for any supported STM32 chip. */
    constexpr uint8_t UART_QUEUE_SIZE   = 10; /**< The max number of independent transmissions that can be stored internally. */
    constexpr uint8_t UART_QUEUE_BUFFER_SIZE =
        32; /**< The max number of bytes that can be stored from a single continuous transmission. */
  }         // namespace UART

  /** @namespace Thor::USART */
  namespace USART
  {
    constexpr uint8_t MAX_USART_CHANNELS = 4; /**< Total possible USART specific channels for any supported STM32 chip. */
    constexpr uint8_t USART_QUEUE_SIZE = 10;  /**< The max number of independent transmissions that can be stored internally. */
    constexpr uint8_t USART_QUEUE_BUFFER_SIZE =
        32; /**< The max number of bytes that can be stored from a single continuous transmission. */
  }         // namespace USART

  /** @namespace Thor::Serial */
  namespace Serial
  {
    constexpr uint8_t MAX_SERIAL_CHANNELS =
        UART::MAX_UART_CHANNELS +
        USART::MAX_USART_CHANNELS;               /**< Total possible UART or USART channels for any supported STM32 chip. */
    constexpr uint32_t BLOCKING_TIMEOUT_MS = 10; /**< Time in mS before a TX or RX in blocking mode will timeout */

    /** Allows mapping of either a USART or UART peripheral to the serial class. This is intended to be internal use only. */
    struct HardwareClassMapping
    {
      bool ON_UART;
      uint8_t peripheral_number;
    };

  }    // namespace Serial

  /** @namespace Thor::Threading */
  namespace Threading
  {
    constexpr uint8_t maxThreads = 15; /**< Maximum number of threads */
    constexpr uint32_t threadInitCheckDelay_ms =
        10; /**< How long to wait during thread initialization before polling to check init complete */
    constexpr uint32_t maxThreadInitTimeout_ms = 1000; /**< Max time to wait for thread init sequence to complete */
  }                                                    // namespace Threading
}    // namespace Thor
#endif /* THOR_DEFINITIONS_H_ */
