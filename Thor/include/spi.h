#pragma once
#ifndef SPI_H_
#define SPI_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/circular_buffer.hpp>


/* Thor Includes */
#include <Thor/include/config.h>
#include <Thor/include/definitions.h>
#include <Thor/include/defaults.h>
#include <Thor/include/interrupt.h>
#include <Thor/include/gpio.h>
#include <Thor/include/ringbuffer.h>
#include <Thor/include/exceptions.h>

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_spi.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_spi.h>
#endif


namespace Thor
{
	namespace Peripheral
	{
		namespace SPI
		{
			class SPIClass
			{
			public:
				#ifdef USING_CHIMERA

				Chimera::SPI::Status init(uint8_t channel, const Chimera::SPI::Setup& setupStruct);


				#endif





				void begin(Thor::Definitions::SPI::SPIOptions options = Thor::Definitions::SPI::NO_OPTIONS);

				Thor::Definitions::SPI::SPIStatus write(uint8_t* val, size_t length = 0, Thor::Definitions::SPI::SPIOptions options = Thor::Definitions::SPI::SS_INACTIVE_AFTER_TX);
				Thor::Definitions::SPI::SPIStatus write(uint8_t* val_in, uint8_t* val_out, size_t length = 0, Thor::Definitions::SPI::SPIOptions options = Thor::Definitions::SPI::SS_INACTIVE_AFTER_TX);


				void end();
				void writeSS(Thor::Definitions::GPIO::LogicLevel state);
				void attachPin(boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> slave_select);
				void detachPin();

				void setSSMode(Thor::Definitions::SPI::SPIOptions ss_mode);
				void attachSettings(SPI_InitTypeDef& settings);
				SPI_InitTypeDef getSettings();
				void reInitialize();

				void setTxModeBlock();
				void setTxModeInterrupt();
				void setTxModeDMA();

				void setRxModeBlock();
				void setRxModeInterrupt();
				void setRxModeDMA();

				void setTxRxModeBlock();
				void setTxRxModeInterrupt();
				void setTxRxModeDMA();

				/*-------------------------------
				* Interrupt Handlers
				*------------------------------*/
				void SPI_IRQHandler();
				void SPI_IRQHandler_TXDMA();
				void SPI_IRQHandler_RXDMA();

				/*-------------------------------
				* Status Flags for User
				*------------------------------*/
				//MOVE TO STRUCT
				bool* isInitialized;
				bool tx_complete = true;
				bool rx_complete = true;
				bool RX_ASYNC;


				uint32_t txMode;
				uint32_t rxMode;
				Thor::Definitions::SPI::SPIOptions SS_ActionAfterTX, SlaveSelectControl;

				/*-------------------------------
				* Buffers
				*------------------------------*/
				struct SPIPacket
				{
					size_t length;
					uint8_t *data_tx;
					uint8_t *data_rx;
					Thor::Definitions::SPI::SPIOptions options;
				} TX_tempPacket;
				boost::circular_buffer<SPIPacket> TXPacketBuffer;

				/* Raw continuous RX packet data */
				uint16_t _rxbuffpckt[Thor::Definitions::SPI::SPI_BUFFER_SIZE];
				SmartBuffer::RingBuffer<uint16_t>* rxBufferedPackets;

				/* Delineates packet size for rxBufferedPackets */
				size_t _rxbuffpcktlen[Thor::Definitions::SPI::SPI_BUFFER_SIZE];
				SmartBuffer::RingBuffer<size_t>* rxBufferedPacketLengths;

				SPIClass(int channel);
				~SPIClass();

			private:
				/*-------------------------------
				* Class Variables / Flags
				*------------------------------*/
				int spi_channel;
				struct SPIClassStatus
				{
					bool gpio_enabled;
					bool spi_enabled;
					bool spi_interrupts_enabled;
					bool dma_enabled_tx;
					bool dma_enabled_rx;
					bool dma_interrupts_enabled_tx;
					bool dma_interrupts_enabled_rx;
				} SPI_PeriphState;

				SPI_HandleTypeDef spi_handle;
				DMA_HandleTypeDef hdma_spi_tx;
				DMA_HandleTypeDef hdma_spi_rx;
				Thor::Definitions::SPI::SPIOptions InterfaceMode, SlaveSelectType;

				IT_Initializer ITSettingsHW, ITSettings_DMA_TX, ITSettings_DMA_RX;

				bool EXT_NSS_ATTACHED;
				boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> EXT_NSS;
				Thor::Definitions::SPI::SPIOptions SLAVE_SELECT_MODE;

				Thor::Peripheral::GPIO::GPIOClass_sPtr MOSI, MISO, SCK, NSS;


				/* SUPER HACKY QUICK FIX FOR DRONE SD LOGGER BUG */
				uint8_t* internalTXBuffer;

				void SPI_Init();
				void SPI_DeInit();
				void SPI_EnableClock(int channel);
				void SPI_DisableClock(int channel);
				void SPI_EnableInterrupts();
				void SPI_DisableInterrupts();

				void SPI_GPIO_Init();
				void SPI_GPIO_DeInit();

				void SPI_DMA_Init_RX();
				void SPI_DMA_Init_TX();
				void SPI_DMA_DeInit_TX();
				void SPI_DMA_DeInit_RX();
				void SPI_DMA_EnableClock();
				void SPI_DMA_EnableInterrupts_TX();
				void SPI_DMA_EnableInterrupts_RX();
				void SPI_DMA_DisableInterrupts_TX();
				void SPI_DMA_DisableInterrupts_RX();
			};
			typedef boost::shared_ptr<SPIClass> SPIClass_sPtr;
		}
	}
}



#ifdef ENABLE_SPI1
extern Thor::Peripheral::SPI::SPIClass_sPtr spi1;
#endif
#ifdef ENABLE_SPI2
extern Thor::Peripheral::SPI::SPIClass_sPtr spi2;
#endif

#ifdef ENABLE_SPI3
extern Thor::Peripheral::SPI::SPIClass_sPtr spi3;
#endif

#ifdef ENABLE_SPI4
extern Thor::Peripheral::SPI::SPIClass_sPtr spi4;
#endif

#ifdef ENABLE_SPI5
extern Thor::Peripheral::SPI::SPIClass_sPtr spi5;
#endif

#ifdef ENABLE_SPI6
extern Thor::Peripheral::SPI::SPIClass_sPtr spi6;
#endif


#ifdef __cplusplus
extern "C" {
#endif

	#ifdef ENABLE_SPI1
	void SPI1_IRQHandler();
	#endif

	#ifdef ENABLE_SPI2
	void SPI2_IRQHandler();
	#endif

	#ifdef ENABLE_SPI3
	void SPI3_IRQHandler();
	#endif

	#ifdef ENABLE_SPI4
	void SPI4_IRQHandler();
	#endif

	#ifdef ENABLE_SPI5
	void SPI5_IRQHandler();
	#endif

	#ifdef ENABLE_SPI6
	void SPI6_IRQHandler();
	#endif

#ifdef __cplusplus
}
#endif


#endif /* SPI_H_*/