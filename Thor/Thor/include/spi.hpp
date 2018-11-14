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

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/defaults.hpp>
#include <Thor/include/interrupt.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/ringbuffer.hpp>
#include <Thor/include/exceptions.hpp>

/* FreeRTOS Includes */
#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

/* Chimera Includes */
#if defined(USING_CHIMERA)
#include <Chimera/interface.hpp>
#endif

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

				/**
				 * @brief Writes a buffer of data
				 * 
				 * @param[in] 	data_in 			Input data buffer that will be written to MOSI
				 * @param[in] 	length 				Number of bytes to be transfered
				 * @param[in]	nssDisableAfterTX 	Optionally disable the chip select line after the transmition is complete
				 * @return Thor::Definitions::Status 
				 */
				Thor::Definitions::Status write(uint8_t * data_in, size_t length = 0, const bool& nssDisableAfterTX = true);
				
				/**
				 * @brief Simultaneously writes and reads data
				 * 
				 * @param[in] 	data_in				Data buffer that will be written to MOSI
				 * @param[out] 	data_out 			Data buffer that will have MISO written to it
				 * @param[in] 	length 				Number of bytes to be transfered
				 * @param[in] 	nssDisableAfterTX 	Optionally disable the chip select line after the transmition is complete
				 * @return Thor::Definitions::Status 
				 */
				Thor::Definitions::Status write(uint8_t * data_in, uint8_t * data_out, size_t length = 0, const bool& nssDisableAfterTX = true);

				/**
				 * @brief De-initializes the SPI peripheral
				 * 
				 * @return void
				 */
				void end();

				/**
				 * @brief Writes the device slave select line high or low
				 * 
				 * @param[in]	state 	The state to drive the chip select line to
				 * @return void
				 */
				void writeSS(Thor::Definitions::GPIO::LogicLevel state);

				/**
				 * @brief Attaches an external pin to be used as the slave select
				 * 
				 * @param[in] 	slave_select 	Reference to the initialized GPIO pin
				 * @return void
				 */
				void attachPin(boost::shared_ptr<Thor::Peripheral::GPIO::GPIOClass> slave_select);

				/**
				 * @brief Removes any previous pin that was attached as the slave select
				 * 
				 * @return void
				 */
				void detachPin();

				/**
				 * @brief Sets the chip select behavior 
				 * 
				 * @param[in]	ss_mode 	Chip select mode
				 * @return void
				 */
				void setSSMode(Thor::Definitions::SPI::Options ss_mode);

				/**
				 * @brief Assigns custom SPI bus settings
				 * 
				 * @param[in]	settings 	User SPI configuration settings
				 * @return void
				 */
				void attachSettings(SPI_InitTypeDef& settings);

				/**
				 * @brief Get the current SPI settings configuration
				 * 
				 * @return SPI_InitTypeDef 
				 */
				SPI_InitTypeDef getSettings();

				/**
				 * @brief Reinitializes the SPI hardware with the current settings
				 * 
				 * @return void
				 */
				void reInitialize();
				
				/** 
				 * 	@brief Place the specified peripheral into a given mode
				 * 
				 *	@param[in]	periph	Explicitly state which peripheral subsystem (TX or RX) to set from Thor::Peripheral::SPI::SubPeripheral
				 *	@param[in] 	mode	The corresponding mode for the peripheral to enter, from Thor::Peripheral::SPI::Modes
				 *	@return Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::SPI::Status
				 **/
				Thor::Definitions::Status setMode(const SubPeripheral& periph, const Modes& mode);

				/** 
				 *  @brief Updates the clock frequency of an already initialized SPI object
				 * 
				 *	@param[in] freq		Desired clock frequency in Hz
				 *	@return Status code indicating peripheral state. Will read 'PERIPH_OK' if everything is fine. Otherwise it
				 *			will return a code from Thor::Peripheral::SPI::Status
				 **/
				Thor::Definitions::Status updateClockFrequency(uint32_t freq);

                uint32_t getClockFrequency();

				/**
				 * @brief Normal interrupt based ISR handler
				 * 
				 * @return void
				 */
				void SPI_IRQHandler();

				/**
				 * @brief DMA TX ISR handler
				 * 
				 * @return void
				 */
				void SPI_IRQHandler_TXDMA();

				/**
				 * @brief DMA RX ISR handler
				 * 
				 * @return void
				 */
				void SPI_IRQHandler_RXDMA();
				
			private:
				/**
				 *  @brief Construct a new SPIClass object
				 * 	
				 * 	This is kept private so users properly manage the class object with 
				 *  shared_ptr instances. 
				 * 	The public constructor is Thor::Peripheral::SPI::SPIClass::create.
				 *  
				 *  @param[in]	channel 	Which hardware peripheral to control with the class
				 */
				SPIClass(const int channel);
				
			public:
				/** 
				 *  @brief A factory method to create a new SPIClass object 
				 *	
				 *	This method intentionally replaces the typical constructor for the purpose of allowing
				 *	the SPI ISR handlers to deduce at runtime which class generated the interrupt. The new 
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
				uint32_t actualClockFrequency = 0;
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

            class ChimeraSPI : public Chimera::SPI::Interface
            {
            public:
                Chimera::SPI::Status begin(const Chimera::SPI::Setup& setupStruct) override;
                Chimera::SPI::Status write(uint8_t* data_in, size_t length, const bool& ssDisableAfterTX = true) override;
                Chimera::SPI::Status write(uint8_t* data_in, uint8_t* data_out, size_t length, const bool& ssDisableAfterTX = true) override;
                Chimera::SPI::Status setMode(Chimera::SPI::SubPeripheral periph, Chimera::SPI::SubPeripheralMode mode) override;
                Chimera::SPI::Status updateClockFrequency(uint32_t freq) override;
                uint32_t getClockFrequency() override;
                void writeSS(Chimera::GPIO::State value) override;

                ChimeraSPI(const int& channel);
                ~ChimeraSPI() = default;

            private:
                int channel;
                boost::shared_ptr<SPIClass> spi;
            };
		}
	}
}

#endif /* SPI_H_*/