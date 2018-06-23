#pragma once
#ifndef THOR_SPI_HPP
#define THOR_SPI_HPP

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/move/unique_ptr.hpp>
#include <boost/circular_buffer.hpp>

/* FreeRTOS Includes */
#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/defaults.hpp>
#include <Thor/include/interrupt.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/ringbuffer.hpp>
#include <Thor/include/exceptions.hpp>


namespace Thor
{
	namespace Peripheral
	{
		namespace SPI
		{
			using namespace Thor::Definitions;
			
			class SPIClass
			{
			public:

				void begin(Thor::Definitions::SPI::Options options = Thor::Definitions::SPI::NO_OPTIONS);

				Thor::Definitions::Status write(uint8_t* data_in, size_t length = 0, const bool& nssDisableAfterTX = true);
				Thor::Definitions::Status write(uint8_t* data_in, uint8_t* data_out, size_t length = 0, const bool& nssDisableAfterTX = true);


				void end();
				void writeSS(Thor::Definitions::GPIO::LogicLevel state);
				void attachPin(boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> slave_select);
				void detachPin();

				void setSSMode(Thor::Definitions::SPI::Options ss_mode);
				void attachSettings(SPI_InitTypeDef& settings);
				SPI_InitTypeDef getSettings();
				void reInitialize();
				
				/** Place the specified peripheral into a given mode
				 *	@param[in] periph	Explicitly states which peripheral subsystem (TX or RX) to set from Thor::Peripheral::SPI::SubPeripheral
				 *	@param[in] mode		The corresponding mode for the peripheral to enter, from Thor::Peripheral::SPI::Modes
				 *	@return Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::SPI::Status
				 **/
				Thor::Definitions::Status setMode(const SubPeripheral& periph, const Modes& mode);

				/** Updates the clock frequency of an already initialized SPI object
				 *	@param[in] freq	Desired clock frequency in Hz
				 *	@return Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::SPI::Status
				 **/
				Thor::Definitions::Status updateClockFrequency(uint32_t freq);

				/*-------------------------------
				* Interrupt Handlers
				*------------------------------*/
				void SPI_IRQHandler();
				void SPI_IRQHandler_TXDMA();
				void SPI_IRQHandler_RXDMA();
				
				static constexpr bool usesBoost = true;
				
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
				static const boost::shared_ptr<SPIClass>& create(const int channel);
				SPIClass() = default;
				~SPIClass() = default;

				/*-------------------------------
				* Buffers
				*------------------------------*/
				struct SPIPacket
				{
					size_t length;
					uint8_t *data_tx;
					uint8_t *data_rx;
					bool disableNSS;
				} TX_tempPacket;
				boost::circular_buffer<SPIPacket> TXPacketBuffer;

				/* Raw continuous RX packet data */
				uint16_t _rxbuffpckt[Thor::Definitions::SPI::SPI_BUFFER_SIZE];
				SmartBuffer::RingBuffer<uint16_t>* rxBufferedPackets;

				/* Delineates packet size for rxBufferedPackets */
				size_t _rxbuffpcktlen[Thor::Definitions::SPI::SPI_BUFFER_SIZE];
				SmartBuffer::RingBuffer<size_t>* rxBufferedPacketLengths;


				

				#ifdef USING_CHIMERA
				Chimera::SPI::Status cbegin(const Chimera::SPI::Setup& setupStruct);
				Chimera::SPI::Status cwrite(uint8_t* in, size_t length, const bool& nssDisableAfterTX);
				Chimera::SPI::Status cwrite(uint8_t* in, uint8_t* out, size_t length, const bool& nssDisableAfterTX);
				Chimera::SPI::Status csetMode(Chimera::SPI::SubPeripheral periph, Chimera::SPI::SubPeripheralMode mode);
				Chimera::SPI::Status cupdateClockFrequency(uint32_t freq);
				void cwriteSS(Chimera::GPIO::State value);
				#endif
				
				#ifdef USING_FREERTOS
				void attachThreadTrigger(Thor::Definitions::Interrupt::Trigger trig, SemaphoreHandle_t* semphr);
				void removeThreadTrigger(Thor::Definitions::Interrupt::Trigger trig);
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
				
				bool _getTXComplete()
				{
					return tx_complete;
				}

				void _setRXComplete()
				{
					rx_complete = true;
				}
				
				bool _getRXComplete()
				{
					return rx_complete;
				}
				
				Thor::Definitions::SPI::Options _getSSControlType(){ return slaveSelectControl; }
				
				bool _getInitStatus(){ return SPI_PeriphState.spi_enabled; }
				
				bool _txBufferEmpty(){ return TXPacketBuffer.empty(); }
				
				void _txBufferRemoveFrontPacket(){ TXPacketBuffer.pop_front(); }
				
				SPIPacket _txBufferNextPacket(){ return TXPacketBuffer.front(); }

			private:
				/*-------------------------------
				* Class Variables / Flags
				*------------------------------*/
				int spi_channel;
				struct SPIClassStatus
				{
					bool gpio_enabled = false;
					bool spi_enabled = false;
					bool spi_interrupts_enabled = false;
					bool dma_enabled_tx = false;
					bool dma_enabled_rx = false;
					bool dma_interrupts_enabled_tx = false;
					bool dma_interrupts_enabled_rx = false;
				} SPI_PeriphState;
				
				Modes txMode = Modes::MODE_UNDEFINED;
				Modes rxMode = Modes::MODE_UNDEFINED;
				
				Thor::Definitions::SPI::Options SS_ActionAfterTX;
				Thor::Definitions::SPI::Options slaveSelectControl = Thor::Definitions::SPI::Options::SS_AUTOMATIC_CONTROL;
				
				bool tx_complete = true;
				bool rx_complete = true;
				bool RX_ASYNC;

				SPI_HandleTypeDef spi_handle;
				DMA_HandleTypeDef hdma_spi_tx;
				DMA_HandleTypeDef hdma_spi_rx;
				Thor::Definitions::SPI::Options InterfaceMode, SlaveSelectType;

				IT_Initializer ITSettingsHW, ITSettings_DMA_TX, ITSettings_DMA_RX;

				bool EXT_NSS_ATTACHED;
				boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> EXT_NSS;
				Thor::Definitions::SPI::Options SLAVE_SELECT_MODE;

				Thor::Peripheral::GPIO::GPIOClass_sPtr MOSI, MISO, SCK, NSS;
				
				/*-------------------------------
				* Low Level Setup/Teardown Functions
				*------------------------------*/
				void SPI_GPIO_Init();
				void SPI_GPIO_DeInit();
				
				void SPI_Init();
				void SPI_DeInit();
				void SPI_EnableClock();
				void SPI_DisableClock();
				void SPI_DMA_EnableClock();
				void SPI_EnableInterrupts();
				void SPI_DisableInterrupts();

				void SPI_DMA_Init(const SubPeripheral& periph);
				void SPI_DMA_DeInit(const SubPeripheral& periph);
				void SPI_DMA_EnableInterrupts(const SubPeripheral& periph);
				void SPI_DMA_DisableInterrupts(const SubPeripheral& periph);
				
				/*-------------------------------
				* Utility Functions
				*------------------------------*/
				bool txRxModesEqual(Modes mode);
			};
			typedef boost::shared_ptr<SPIClass> SPIClass_sPtr;
			typedef boost::movelib::unique_ptr<SPIClass> SPIClass_uPtr;

		}
	}
}

#endif /* SPI_H_*/