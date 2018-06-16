#pragma once
#ifndef THOR_DEFINITIONS_H_
#define THOR_DEFINITIONS_H_

#include <Thor/include/config.hpp>

#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif 

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Definitions */
	namespace Definitions
	{
		enum class Status : int
		{
			PERIPH_TIMEOUT                     = -5,
			PERIPH_LOCKED                      = -4,
			PERIPH_NOT_INITIALIZED             = -3,
			PERIPH_ERROR                       = -2,
			PERIPH_NOT_READY                   = -1,
			PERIPH_OK                          = 0,
			PERIPH_READY,
			PERIPH_INVALID_PARAM,
			PERIPH_TX_IN_PROGRESS,
			PERIPH_RX_IN_PROGRESS,
			PERIPH_PACKET_TOO_LARGE_FOR_BUFFER,
			PERIPH_PACKET_NONE_AVAILABLE
		};

		enum class SubPeripheral : uint8_t
		{
			RX,
			TX,
			TXRX
		};
		
		enum class Modes : uint8_t
		{
			MODE_UNDEFINED,
			BLOCKING,
			INTERRUPT,
			DMA
		};

		enum class ClockBus : uint8_t
		{
			APB1_PERIPH,
			APB2_PERIPH,
			APB1_TIMER,
			APB2_TIMER
		};

		
		/** @namespace Thor::Definitions::Interrupt */
		namespace Interrupt
		{
			#if defined(USING_FREERTOS)
			const uint32_t EXTI0_MAX_IRQn_PRIORITY = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
			const uint32_t MAX_PENDING_TASK_TRIGGERS = 10;			/**< The largest number of queued events at any given time */
			
			/** The various types of triggers that can be used to unlock a FreeRTOS thread */
			enum Trigger : uint8_t
			{
				RX_COMPLETE,
				TX_COMPLETE,
				TXRX_COMPLETE,
				BUFFERED_TX_COMPLETE,
				BUFFERED_TXRX_COMPLETE,
				MAX_SOURCES
			};
			#endif
		}
		
		/** @namespace Thor::Definitions::GPIO */
		namespace GPIO
		{
			/** Different possible phrasings of instructing a pin to turn on/off */
			enum LogicLevel : bool
			{
				LOW      = false,
				OFF      = false,
				ZERO     = false,
				DISABLED = false,
				HIGH     = true,
				ON       = true,
				ONE      = true,
				ENABLED  = true
			};
		}

		/** @namespace Thor::Definitions::TIMER */
		namespace TIMER
		{
			const unsigned int MAX_CHANNELS = 16;
			const unsigned int MAX_SUB_CHANNELS = 6;
			const unsigned int MAX_ALT_PORTS = 4;
			const unsigned int MAX_ALT_PINS = 4;

			/*--------------------------
			* Hardware Descriptors
			*--------------------------*/
			const uint32_t timerBaseAddresses[] =
			{
				#if defined(STM32F446xx) || defined(STM32F767xx)
				0,
				/* Indexing offset since no TIM0 */
				TIM1_BASE,
				TIM2_BASE,
				TIM3_BASE,
				TIM4_BASE,
				TIM5_BASE,
				TIM6_BASE,
				TIM7_BASE,
				TIM8_BASE,
				TIM9_BASE,
				TIM10_BASE,
				TIM11_BASE,
				TIM12_BASE,
				TIM13_BASE,
				TIM14_BASE
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
				TIMER_BASIC_CHANNELS               = 2u,
				TIMER_BASIC_SUB_CHANNELS           = 1u,

				TIMER_GENERAL_PURPOSE_CHANNELS     = 10u,
				TIMER_GENERAL_PURPOSE_SUB_CHANNELS = 4u,

				TIMER_ADVANCED_CHANNELS            = 2u,
				TIMER_ADVANCED_SUB_CHANNELS        = 6u,

				TIMER_LOW_POWER_CHANNELS           = 1u,
				TIMER_LOW_POWER_SUB_CHANNELS       = 1u
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
				TIMER_DIRECTION_ALL = (TIMER_UP | TIMER_DOWN | TIMER_AUTO_RELOAD)
			};

			enum TimerModes
			{
				TIMER_INPUT_CAPTURE  = 1u,
				TIMER_OUTPUT_COMPARE = 2u,
				TIMER_PWM            = 4u,
				TIMER_ONE_PULSE      = 8u,
				TIMER_ENCODER        = 16u,
				TIMER_BASE           = 32u,
				TIMER_MODE_TIER_1    = (TIMER_INPUT_CAPTURE | TIMER_OUTPUT_COMPARE | TIMER_PWM | TIMER_ONE_PULSE | TIMER_BASE | TIMER_ENCODER),
				TIMER_MODE_TIER_2    = (TIMER_INPUT_CAPTURE | TIMER_OUTPUT_COMPARE | TIMER_PWM | TIMER_ONE_PULSE | TIMER_BASE),
				TIMER_MODE_TIER_3    = (TIMER_PWM | TIMER_ONE_PULSE | TIMER_BASE | TIMER_ENCODER)
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
		}

		/** @namespace Thor::Definitions::SPI */
		namespace SPI
		{
			const unsigned int MAX_SPI_CHANNELS = 6;
			const unsigned int SPI_BUFFER_SIZE = 32;
			const uint32_t BLOCKING_TIMEOUT_MS = 10;	/**< Time in mS before a TX or RX in blocking mode will timeout */
			
			enum Options
			{
				NO_OPTIONS            = 0u,
				MASTER                = (1u << 0),
				SLAVE                 = (1u << 1),
				INTERNAL_SLAVE_SELECT = (1u << 2),
				EXTERNAL_SLAVE_SELECT = (1u << 3),
				SS_ACTIVE_AFTER_TX    = (1u << 4),
				SS_INACTIVE_AFTER_TX  = (1u << 5),
				SS_PULSE              = (1u << 6),
				SS_MANUAL_CONTROL     = (1u << 7),
				SS_AUTOMATIC_CONTROL  = (1u << 8)
			};
		}

		/** @namespace Thor::Definitions::DMA */
		namespace DMA
		{
			/* Useful Macros for Generating DMA Register Addresses*/
			#define DMA_OFFSET_LISR 0x00U
			#define DMA_OFFSET_HISR 0x04U
			#define DMA_OFFSET_LIFCR 0x08U
			#define DMA_OFFSET_HIFCR 0x0CU
			#define DMA_OFFSET_SxCR(STREAM_NUMBER)		(0x10U + 0x18U*(uint32_t)STREAM_NUMBER)
			#define DMA_OFFSET_SxNDTR(STREAM_NUMBER)	(0x14U + 0x18U*(uint32_t)STREAM_NUMBER)
			#define DMA_OFFSET_SxPAR(STREAM_NUMBER)		(0x18U + 0x18U*(uint32_t)STREAM_NUMBER)
			#define DMA_OFFSET_SxM0AR(STREAM_NUMBER)	(0x1CU + 0x18U*(uint32_t)STREAM_NUMBER)
			#define DMA_OFFSET_SxM1AR(STREAM_NUMBER)	(0x20U + 0x18U*(uint32_t)STREAM_NUMBER)

			//The data sheet has conflicting address definitions for this register. The register map
			//does not match manual calculations using the equation below...unsure which is right
			#define DMA_OFFSET_SxFCR(STREAM_NUMBER)		(0x24U + 0x24U*(uint32_t)STREAM_NUMBER)

			/* Register Definitions for DMA1 */
			#define DMA1_LISR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_LISR))
			#define DMA1_HISR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_HISR))
			#define DMA1_LIFCR  (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_LIFCR))
			#define DMA1_HIFCR  (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_HIFCR))
			#define DMA1_S0CR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxCR(0)))
			#define DMA1_S1CR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxCR(1)))		/* DMA1_SxCR */
			#define DMA1_S2CR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxCR(2)))
			#define DMA1_S3CR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxCR(3)))
			#define DMA1_S4CR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxCR(4)))
			#define DMA1_S5CR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxCR(5)))
			#define DMA1_S6CR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxCR(6)))
			#define DMA1_S7CR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxCR(7)))
			#define DMA1_S0NDTR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxNDTR(0)))
			#define DMA1_S1NDTR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxNDTR(1)))	/* DMA1_SxNDTR */
			#define DMA1_S2NDTR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxNDTR(2)))
			#define DMA1_S3NDTR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxNDTR(3)))
			#define DMA1_S4NDTR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxNDTR(4)))
			#define DMA1_S5NDTR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxNDTR(5)))
			#define DMA1_S6NDTR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxNDTR(6)))
			#define DMA1_S7NDTR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxNDTR(7)))
			#define DMA1_S0PAR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxPAR(0)))
			#define DMA1_S1PAR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxPAR(1)))		/* DMA1_SxPAR */
			#define DMA1_S2PAR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxPAR(2)))
			#define DMA1_S3PAR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxPAR(3)))
			#define DMA1_S4PAR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxPAR(4)))
			#define DMA1_S5PAR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxPAR(5)))
			#define DMA1_S6PAR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxPAR(6)))
			#define DMA1_S7PAR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxPAR(7)))
			#define DMA1_S0M0AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM0AR(0)))
			#define DMA1_S1M0AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM0AR(1)))	/* DMA1_SxM0AR */
			#define DMA1_S2M0AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM0AR(2)))
			#define DMA1_S3M0AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM0AR(3)))
			#define DMA1_S4M0AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM0AR(4)))
			#define DMA1_S5M0AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM0AR(5)))
			#define DMA1_S6M0AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM0AR(6)))
			#define DMA1_S7M0AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM0AR(7)))
			#define DMA1_S0M1AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM1AR(0)))
			#define DMA1_S1M1AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM1AR(1)))	/* DMA1_SxM1AR*/
			#define DMA1_S2M1AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM1AR(2)))
			#define DMA1_S3M1AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM1AR(3)))
			#define DMA1_S4M1AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM1AR(4)))
			#define DMA1_S5M1AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM1AR(5)))
			#define DMA1_S6M1AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM1AR(6)))
			#define DMA1_S7M1AR (*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxM1AR(7)))
			#define DMA1_S0FCR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxFCR(0)))
			#define DMA1_S1FCR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxFCR(1)))		/* DMA1_SxFCR */
			#define DMA1_S2FCR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxFCR(2)))
			#define DMA1_S3FCR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxFCR(3)))
			#define DMA1_S4FCR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxFCR(4)))
			#define DMA1_S5FCR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxFCR(5)))
			#define DMA1_S6FCR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxFCR(6)))
			#define DMA1_S7FCR	(*(uint32_t*)(DMA1_BASE + DMA_OFFSET_SxFCR(7)))

			/* Register Definitions for DMA2 */
			#define DMA2_LISR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_LISR))
			#define DMA2_HISR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_HISR))
			#define DMA2_LIFCR  (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_LIFCR))
			#define DMA2_HIFCR  (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_HIFCR))
			#define DMA2_S0CR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxCR(0)))
			#define DMA2_S1CR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxCR(1)))		/* DMA2_SxCR */
			#define DMA2_S2CR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxCR(2)))
			#define DMA2_S3CR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxCR(3)))
			#define DMA2_S4CR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxCR(4)))
			#define DMA2_S5CR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxCR(5)))
			#define DMA2_S6CR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxCR(6)))
			#define DMA2_S7CR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxCR(7)))
			#define DMA2_S0NDTR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxNDTR(0)))
			#define DMA2_S1NDTR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxNDTR(1)))	/* DMA2_SxNDTR */
			#define DMA2_S2NDTR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxNDTR(2)))
			#define DMA2_S3NDTR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxNDTR(3)))
			#define DMA2_S4NDTR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxNDTR(4)))
			#define DMA2_S5NDTR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxNDTR(5)))
			#define DMA2_S6NDTR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxNDTR(6)))
			#define DMA2_S7NDTR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxNDTR(7)))
			#define DMA2_S0PAR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxPAR(0)))
			#define DMA2_S1PAR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxPAR(1)))		/* DMA2_SxPAR */
			#define DMA2_S2PAR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxPAR(2)))
			#define DMA2_S3PAR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxPAR(3)))
			#define DMA2_S4PAR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxPAR(4)))
			#define DMA2_S5PAR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxPAR(5)))
			#define DMA2_S6PAR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxPAR(6)))
			#define DMA2_S7PAR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxPAR(7)))
			#define DMA2_S0M0AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM0AR(0)))
			#define DMA2_S1M0AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM0AR(1)))	/* DMA2_SxM0AR */
			#define DMA2_S2M0AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM0AR(2)))
			#define DMA2_S3M0AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM0AR(3)))
			#define DMA2_S4M0AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM0AR(4)))
			#define DMA2_S5M0AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM0AR(5)))
			#define DMA2_S6M0AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM0AR(6)))
			#define DMA2_S7M0AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM0AR(7)))
			#define DMA2_S0M1AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM1AR(0)))
			#define DMA2_S1M1AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM1AR(1)))	/* DMA2_SxM1AR*/
			#define DMA2_S2M1AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM1AR(2)))
			#define DMA2_S3M1AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM1AR(3)))
			#define DMA2_S4M1AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM1AR(4)))
			#define DMA2_S5M1AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM1AR(5)))
			#define DMA2_S6M1AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM1AR(6)))
			#define DMA2_S7M1AR (*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxM1AR(7)))
			#define DMA2_S0FCR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxFCR(0)))
			#define DMA2_S1FCR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxFCR(1)))		/* DMA2_SxFCR */
			#define DMA2_S2FCR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxFCR(2)))
			#define DMA2_S3FCR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxFCR(3)))
			#define DMA2_S4FCR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxFCR(4)))
			#define DMA2_S5FCR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxFCR(5)))
			#define DMA2_S6FCR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxFCR(6)))
			#define DMA2_S7FCR	(*(uint32_t*)(DMA2_BASE + DMA_OFFSET_SxFCR(7)))

			enum TransferDirection
			{
				PERIPH_TO_MEM,
				MEM_TO_PERIPH,
				MEM_TO_MEM,
				TRANSFER_DIRECTION_UNDEFINED
			};
		}

		/** @namespace Thor::Definitions::UART */
		namespace UART
		{
			const unsigned int MAX_UART_CHANNELS = 4;			/**< Total possible UART specific channels for any supported STM32 chip. */
			const unsigned int UART_QUEUE_SIZE = 10;				/**< The max number of independent transmissions that can be stored internally. */
			const unsigned int UART_QUEUE_BUFFER_SIZE = 32;		/**< The max number of bytes that can be stored from a single continuous transmission. */
		}

		/** @namespace Thor::Definitions::USART */
		namespace USART
		{
			const unsigned int MAX_USART_CHANNELS = 4; /**< Total possible USART specific channels for any supported STM32 chip. */
		}

		/** @namespace Thor::Definitions::Serial */
		namespace Serial
		{
			const unsigned int MAX_SERIAL_CHANNELS = Thor::Definitions::UART::MAX_UART_CHANNELS + Thor::Definitions::USART::MAX_USART_CHANNELS; /**< Total possible UART or USART channels for any supported STM32 chip. */
			const uint32_t BLOCKING_TIMEOUT_MS = 10;	/**< Time in mS before a TX or RX in blocking mode will timeout */

			/** Supported communication baudrates */
			enum BaudRate : uint32_t
			{
				SERIAL_BAUD_110    = 100u,
				SERIAL_BAUD_150    = 150u,
				SERIAL_BAUD_300    = 300u,
				SERIAL_BAUD_1200   = 1200u,
				SERIAL_BAUD_2400   = 2400u,
				SERIAL_BAUD_4800   = 4800u,
				SERIAL_BAUD_9600   = 9600u,
				SERIAL_BAUD_19200  = 19200u,
				SERIAL_BAUD_38400  = 38400u,
				SERIAL_BAUD_57600  = 57600u,
				SERIAL_BAUD_115200 = 115200u,
				SERIAL_BAUD_230400 = 230400u,
				SERIAL_BAUD_460800 = 460800u,
				SERIAL_BAUD_921600 = 921600u
			};

			/** Allows mapping of either a USART or UART peripheral to the serial class. This is intended to be internal use only. */
			struct HardwareClassMapping
			{
				bool ON_UART;
				uint8_t peripheral_number;
			};
			
			class SerialBase
			{
			public:
				virtual Status begin(const BaudRate&, const Modes&, const Modes&) = 0;
				virtual Status setMode(const SubPeripheral&, const Modes&) = 0;
				virtual Status write(uint8_t*, size_t) = 0;
				virtual Status write(char*, size_t) = 0;
				virtual Status write(const char*) = 0;
				virtual Status write(const char*, size_t) = 0;
				virtual Status readSync(uint8_t*, size_t) = 0;
				virtual Status readPacket(uint8_t*, size_t) = 0;
				//virtual Status readBytes(uint8_t*, size_t) = 0;
				virtual uint32_t availablePackets() = 0;
				virtual size_t nextPacketSize() = 0;
				virtual void end() = 0;
				
				#if defined(USING_FREERTOS)
				virtual void attachThreadTrigger(Thor::Definitions::Interrupt::Trigger, SemaphoreHandle_t*) = 0;
				virtual void removeThreadTrigger(Thor::Definitions::Interrupt::Trigger) = 0;
				#endif 
				
			private:	
			};
		}
		
		/** @namespace Thor::Definitions::Threading */
		namespace Threading
		{
			const uint8_t maxThreads = 15;					/**< Maximum number of threads */
			const uint32_t threadInitCheckDelay_ms = 10;	/**< How long to wait during thread initialization before polling to check init complete */
			const uint32_t maxThreadInitTimeout_ms = 1000;  /**< Max time to wait for thread init sequence to complete */
		}
	}
}
#endif /* THOR_DEFINITIONS_H_ */