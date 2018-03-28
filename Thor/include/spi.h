#pragma once
#ifndef SPI_H_
#define SPI_H_

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_spi.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_spi.h>
#endif

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Thor Includes */
#include "thor_config.h"
#include "thor_definitions.h"
#include "interrupt.h"
#include "defaults.h"
#include "gpio.h"
#include "ringbuffer.h"
#include "exceptions.h"

/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/circular_buffer.hpp>

class SPIClass;
typedef boost::shared_ptr<SPIClass> SPIClass_sPtr;



class SPIClass
{
public:
	void begin(ThorDef::SPI::SPIOptions options = ThorDef::SPI::NO_OPTIONS);

	ThorDef::SPI::SPIStatus write(uint8_t* val, size_t length = 0, ThorDef::SPI::SPIOptions options = ThorDef::SPI::SS_INACTIVE_AFTER_TX);
	ThorDef::SPI::SPIStatus write(uint8_t* val_in, uint8_t* val_out, size_t length = 0, ThorDef::SPI::SPIOptions options = ThorDef::SPI::SS_INACTIVE_AFTER_TX);
	
	
	void end();
	void writeSS(ThorDef::GPIO::LogicLevel state);
	void attachPin(boost::shared_ptr<GPIOClass> slave_select);
	void detachPin();

	void setSSMode(ThorDef::SPI::SPIOptions ss_mode);
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
	ThorDef::SPI::SPIOptions SS_ActionAfterTX, SlaveSelectControl;

	/*-------------------------------
	* Buffers
	*------------------------------*/
	struct SPIPacket
	{
		size_t length;
		uint8_t *data_tx;
		uint8_t *data_rx;
		ThorDef::SPI::SPIOptions options;
	} TX_tempPacket;
	boost::circular_buffer<SPIPacket> TXPacketBuffer;

	/* Raw continuous RX packet data */
	uint16_t _rxbuffpckt[ThorDef::SPI::SPI_BUFFER_SIZE];
	SmartBuffer::RingBuffer<uint16_t>* rxBufferedPackets;
	
	/* Delineates packet size for rxBufferedPackets */
	size_t _rxbuffpcktlen[ThorDef::SPI::SPI_BUFFER_SIZE];
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
	ThorDef::SPI::SPIOptions InterfaceMode, SlaveSelectType;

	IT_Initializer ITSettingsHW, ITSettings_DMA_TX, ITSettings_DMA_RX;

	bool EXT_NSS_ATTACHED;
	boost::shared_ptr<GPIOClass> EXT_NSS;
	ThorDef::SPI::SPIOptions SLAVE_SELECT_MODE;

	GPIOClass_sPtr MOSI, MISO, SCK, NSS;
	

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

#ifdef ENABLE_SPI1
extern SPIClass_sPtr spi1;
#endif
#ifdef ENABLE_SPI2
extern SPIClass_sPtr spi2;
#endif

#ifdef ENABLE_SPI3
extern SPIClass_sPtr spi3;
#endif

#ifdef ENABLE_SPI4
extern SPIClass_sPtr spi4;
#endif

#ifdef ENABLE_SPI5
extern SPIClass_sPtr spi5;
#endif

#ifdef ENABLE_SPI6
extern SPIClass_sPtr spi6;
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