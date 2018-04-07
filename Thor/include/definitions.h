#pragma once
#ifndef THOR_DEFINITIONS_H_
#define THOR_DEFINITIONS_H_

#include <Thor/include/config.h>

#include <boost/container/flat_map.hpp>

namespace Thor
{
	namespace Definitions
	{
		namespace GPIO
		{
			enum LogicLevel : bool
			{
				LOW = false,
				OFF = false,
				ZERO = false,
				DISABLED = false,
				HIGH = true,
				ON = true,
				ONE = true,
				ENABLED = true
			};


			extern boost::container::flat_map<GPIO_TypeDef*, uint32_t> rcc_gpio_mask;

			
		}
		
		namespace TIMER
		{
			#ifdef TARGET_STM32F7

			#endif
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
			
		namespace SPI
		{
			const unsigned int MAX_SPI_CHANNELS = 6;
			const unsigned int SPI_BUFFER_SIZE = 32;
		
			enum TxRxModes
			{
				TX_MODE_NONE,
				TX_MODE_BLOCKING,
				TX_MODE_INTERRUPT,
				TX_MODE_DMA,
				RX_MODE_NONE,
				RX_MODE_BLOCKING,
				RX_MODE_INTERRUPT,
				RX_MODE_DMA,
				TXRX_MODE_BLOCKING,
				TXRX_MODE_INTERRUPT,
				TXRX_MODE_DMA
			};
		
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

			enum Status
			{
				SPI_NOT_INITIALIZED = -3,
				SPI_ERROR           = -2,
				SPI_NOT_READY       = -1,
				SPI_READY           = 0,
				SPI_TX_BUSY,
				SPI_RX_OK
			};
		}
		
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
	
			/* Register Definitions for DMA1 */
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
			}
			;
	
		
	
		}

		namespace Serial
		{
			const unsigned int MAX_SERIAL_CHANNELS = 8;
			const unsigned int MAX_UART_CHANNELS = 4;
			const unsigned int UART_BUFFER_SIZE = 32;
			const unsigned int UART_PACKET_QUEUE_SIZE = 10;

			enum UARTCommunicationModes
			{
				TX_MODE_NONE,
				TX_MODE_BLOCKING,
				TX_MODE_INTERRUPT,
				TX_MODE_DMA,
				RX_MODE_NONE,
				RX_MODE_BLOCKING,
				RX_MODE_INTERRUPT,
				RX_MODE_DMA,

			};

			typedef enum
			{
				SERIAL_BAUD_110 = 100u,
				SERIAL_BAUD_150 = 150u,
				SERIAL_BAUD_300 = 300u,
				SERIAL_BAUD_1200 = 1200u,
				SERIAL_BAUD_2400 = 2400u,
				SERIAL_BAUD_4800 = 4800u,
				SERIAL_BAUD_9600 = 9600u,
				SERIAL_BAUD_19200 = 19200u,
				SERIAL_BAUD_38400 = 38400u,
				SERIAL_BAUD_57600 = 57600u,
				SERIAL_BAUD_115200 = 115200u,
				SERIAL_BAUD_230400 = 230400u,
				SERIAL_BAUD_460800 = 460800u,
				SERIAL_BAUD_921600 = 921600u
			} BaudRate;

		}
	}
	
	//TODO: Deprecate this!
	namespace Libraries
	{
		//TODO: Deprecate this!
		namespace SD
		{
			/* All of these definitions in the SD namespace were taken from the
			 * stm32_adafruit_sd.h/.c files from a project generated with STMicro
			 * CubeMX software. They are slightly modified to fit with the structure
			 * needed in this library. */
			enum {
				BSP_SD_OK      = 0x00,
				MSD_OK         = 0x00,
				BSP_SD_ERROR   = 0x01,
				BSP_SD_TIMEOUT
			};

			typedef struct
			{
				uint8_t  Reserved1 : 2; /* Reserved */
				uint16_t DeviceSize : 12; /* Device Size */
				uint8_t  MaxRdCurrentVDDMin : 3; /* Max. read current @ VDD min */
				uint8_t  MaxRdCurrentVDDMax : 3; /* Max. read current @ VDD max */
				uint8_t  MaxWrCurrentVDDMin : 3; /* Max. write current @ VDD min */
				uint8_t  MaxWrCurrentVDDMax : 3; /* Max. write current @ VDD max */
				uint8_t  DeviceSizeMul : 3; /* Device size multiplier */
			} struct_v1;

			typedef struct
			{
				uint8_t  Reserved1 : 6; /* Reserved */
				uint32_t DeviceSize : 22; /* Device Size */
				uint8_t  Reserved2 : 1; /* Reserved */
			} struct_v2;

			typedef struct
			{
				/* Card Specific Data: CSD Register */
				/* Header part */
				uint8_t  CSDStruct : 2; /* CSD structure */
				uint8_t  Reserved1 : 6; /* Reserved */
				uint8_t  TAAC : 8; /* Data read access-time 1 */
				uint8_t  NSAC : 8; /* Data read access-time 2 in CLK cycles */
				uint8_t  MaxBusClkFrec : 8; /* Max. bus clock frequency */
				uint16_t CardComdClasses : 12; /* Card command classes */
				uint8_t  RdBlockLen : 4; /* Max. read data block length */
				uint8_t  PartBlockRead : 1; /* Partial blocks for read allowed */
				uint8_t  WrBlockMisalign : 1; /* Write block misalignment */
				uint8_t  RdBlockMisalign : 1; /* Read block misalignment */
				uint8_t  DSRImpl : 1; /* DSR implemented */

												   /* v1 or v2 struct */
				union csd_version {
					struct_v1 v1;
					struct_v2 v2;
				} version;

				uint8_t  EraseSingleBlockEnable : 1; /* Erase single block enable */
				uint8_t  EraseSectorSize : 7; /* Erase group size multiplier */
				uint8_t  WrProtectGrSize : 7; /* Write protect group size */
				uint8_t  WrProtectGrEnable : 1; /* Write protect group enable */
				uint8_t  Reserved2 : 2; /* Reserved */
				uint8_t  WrSpeedFact : 3; /* Write speed factor */
				uint8_t  MaxWrBlockLen : 4; /* Max. write data block length */
				uint8_t  WriteBlockPartial : 1; /* Partial blocks for write allowed */
				uint8_t  Reserved3 : 5; /* Reserved */
				uint8_t  FileFormatGrouop : 1; /* File format group */
				uint8_t  CopyFlag : 1; /* Copy flag (OTP) */
				uint8_t  PermWrProtect : 1; /* Permanent write protection */
				uint8_t  TempWrProtect : 1; /* Temporary write protection */
				uint8_t  FileFormat : 2; /* File Format */
				uint8_t  Reserved4 : 2; /* Reserved */
				uint8_t  crc : 7; /* Reserved */
				uint8_t  Reserved5 : 1; /* always 1*/

			} SD_CSD;
		
			typedef struct
			{
				/* Card Identification Data: CID Register */
				__IO uint8_t  ManufacturerID; /* ManufacturerID */
				__IO uint16_t OEM_AppliID; /* OEM/Application ID */
				__IO uint32_t ProdName1; /* Product Name part1 */
				__IO uint8_t  ProdName2; /* Product Name part2*/
				__IO uint8_t  ProdRev; /* Product Revision */
				__IO uint32_t ProdSN; /* Product Serial Number */
				__IO uint8_t  Reserved1; /* Reserved1 */
				__IO uint16_t ManufactDate; /* Manufacturing Date */
				__IO uint8_t  CID_CRC; /* CID CRC */
				__IO uint8_t  Reserved2; /* always 1 */
			} SD_CID;

			typedef struct
			{
				SD_CSD Csd;
				SD_CID Cid;
				uint32_t CardCapacity; /*!< Card Capacity */
				uint32_t CardBlockSize; /*!< Card Block Size */
				uint32_t LogBlockNbr; /*!< Specifies the Card logical Capacity in blocks   */
				uint32_t LogBlockSize; /*!< Specifies logical block size in bytes           */
			} SD_CardInfo;

			const unsigned int SD_BLOCK_SIZE	= 0x200;
			const unsigned int SD_PRESENT		= 0x01;
			const unsigned int SD_NOT_PRESENT	= 0x00;
			const unsigned int SD_DATATIMEOUT	= 100000000;
		}
		
		
		
	}
	

}
#endif /* THOR_DEFINITIONS_H_ */