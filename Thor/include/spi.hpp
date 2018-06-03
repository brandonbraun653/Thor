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
#include <boost/enable_shared_from_this.hpp>

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/defaults.hpp>
#include <Thor/include/interrupt.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/ringbuffer.hpp>
#include <Thor/include/exceptions.hpp>

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

				void begin(Thor::Definitions::SPI::Options options = Thor::Definitions::SPI::NO_OPTIONS);

				Thor::Definitions::SPI::Status write(uint8_t* val, size_t length = 0, Thor::Definitions::SPI::Options options = Thor::Definitions::SPI::SS_INACTIVE_AFTER_TX);
				Thor::Definitions::SPI::Status write(uint8_t* val_in, uint8_t* val_out, size_t length = 0, Thor::Definitions::SPI::Options options = Thor::Definitions::SPI::SS_INACTIVE_AFTER_TX);


				void end();
				void writeSS(Thor::Definitions::GPIO::LogicLevel state);
				void attachPin(boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> slave_select);
				void detachPin();

				void setSSMode(Thor::Definitions::SPI::Options ss_mode);
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
				
				
			private:
				/** The real constructor used by SPIClass::create */
				SPIClass(const int channel);
				
			public:
				/** A factory method to create a new SPIClass object 
				 *	
				 *	This method intentionally replaces the typical constructor for the purpose of allowing
				 *	the SPI ISR handlers to deduce at runtim which class generated the interrupt. The new 
				 *	object is internally registered with a static vector that keeps track of this.
				 *	
				 *	@param[in] channel Hardware SPI peripheral channel number (i.e. 1 for SPI1, 2 for SPI2, etc)
				 *	@return Shared pointer to the new object
				 **/
				static boost::shared_ptr<SPIClass> create(const int channel);
				~SPIClass() = default;

				/*-------------------------------
				* Status Flags for User
				*------------------------------*/
				bool* isInitialized;
				bool tx_complete = true;
				bool rx_complete = true;
				bool RX_ASYNC;


				uint32_t txMode;
				uint32_t rxMode;
				Thor::Definitions::SPI::Options SS_ActionAfterTX, SlaveSelectControl;

				/*-------------------------------
				* Buffers
				*------------------------------*/
				struct SPIPacket
				{
					size_t length;
					uint8_t *data_tx;
					uint8_t *data_rx;
					Thor::Definitions::SPI::Options options;
				} TX_tempPacket;
				boost::circular_buffer<SPIPacket> TXPacketBuffer;

				/* Raw continuous RX packet data */
				uint16_t _rxbuffpckt[Thor::Definitions::SPI::SPI_BUFFER_SIZE];
				SmartBuffer::RingBuffer<uint16_t>* rxBufferedPackets;

				/* Delineates packet size for rxBufferedPackets */
				size_t _rxbuffpcktlen[Thor::Definitions::SPI::SPI_BUFFER_SIZE];
				SmartBuffer::RingBuffer<size_t>* rxBufferedPacketLengths;


				

				#ifdef USING_CHIMERA
				Chimera::SPI::Status init(int channel, const Chimera::SPI::Setup& setupStruct);
				Chimera::SPI::Status write(uint8_t* in, size_t length);
				Chimera::SPI::Status write(uint8_t* in, uint8_t* out, size_t length);

				Chimera::SPI::Status setTxMode(Chimera::SPI::TXRXMode mode);
				Chimera::SPI::Status setRxMode(Chimera::SPI::TXRXMode mode);
				#endif
				
				/*-------------------------------
				* ISR Stubs 
				*------------------------------*/
				int _getChannel()
				{
					return spi_channel;
				}
				
				void _setTXComplete()
				{
					tx_complete = true;
				}
				
				void _setRXComplete()
				{
					rx_complete = true;
				}
				
				bool _getRXComplete()
				{
					return rx_complete;
				}
				
				bool _getInitStatus(){ return SPI_PeriphState.spi_enabled; }
				
				bool _txBufferEmpty(){ return TXPacketBuffer.empty(); }
				
				void _txBufferRemoveFrontPacket(){ TXPacketBuffer.pop_front(); }
				
				const SPIPacket& _txBufferNextPacket(){ return TXPacketBuffer.front(); }

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
				Thor::Definitions::SPI::Options InterfaceMode, SlaveSelectType;

				IT_Initializer ITSettingsHW, ITSettings_DMA_TX, ITSettings_DMA_RX;

				bool EXT_NSS_ATTACHED;
				boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> EXT_NSS;
				Thor::Definitions::SPI::Options SLAVE_SELECT_MODE;

				Thor::Peripheral::GPIO::GPIOClass_sPtr MOSI, MISO, SCK, NSS;

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


			extern void attachSPI(int channel, SPIClass* spi_class);
		}
	}
}

#endif /* SPI_H_*/