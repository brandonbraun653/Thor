#pragma once
#ifndef THOR_SPI_HPP
#define THOR_SPI_HPP

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <limits>
#include <type_traits>

/* Boost Includes */
#include <boost/static_assert.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/move/unique_ptr.hpp>

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
            class SPIClass;
            class ChimeraSPI;

            typedef boost::shared_ptr<SPIClass> SPIClass_sPtr;
			typedef boost::movelib::unique_ptr<SPIClass> SPIClass_uPtr;

            struct Config
            {
                GPIO::PinConfig SCK;
                GPIO::PinConfig MOSI;
                GPIO::PinConfig MISO;
                GPIO::PinConfig CS;

                SPI_InitTypeDef settings;
            };

			class SPIClass
			{
                friend class ChimeraSPI;

              public:

                /**
				 *  @brief A factory method to create a new SPIClass object
				 *
				 *	This method intentionally replaces the typical constructor for the purpose of allowing
				 *	the SPI ISR handlers to deduce at runtime which class generated the interrupt. The new
				 *	object is internally registered with a static vector that keeps track of this.
				 *
				 *	@param[in] channel Hardware SPI peripheral channel number (i.e. 1 for SPI1, 2 for SPI2, etc)
				 *	@return const boost::shared_ptr<SPIClass>&
				 **/
                static const SPIClass_sPtr create(const uint8_t &channel);

                /**
                 *  @brief Configure the SPI channel to the given settings.
                 *
                 *  If the channel has already been initialized, the settings will only apply if the
                 *  force parameter is set to true.
                 *
                 *  @param[in]  settings    Configuration options for the SPI channel
                 *  @param[in]  force       Forcefully override any existing SPI channel settings
                 *  @return Thor::Definitions::Status
                 */
                Thor::Definitions::Status begin(const Config& settings, const bool &force = false);

                /**
				 *  @brief De-initializes the SPI peripheral
                 *
                 *  If this is the last reference to the channel, the entire channel will be destroyed at the hardware level
				 *
				 *  @return Thor::Definitions::Status
				 */
				Thor::Definitions::Status end();

                /**
				 *  @brief Attempts to reserve the hardware this SPIClass is tied to.
                 *
                 *  Assuming a lock is achieved, the class will hold complete ownership of the hardware until it explicitly
                 *  released it or the auto-unlock feature is used.
				 *
                 *  @param[in]  timeout_ms  The number of milliseconds to wait to achieve a lock
				 *  @return Thor::Definitions::Status
				 */
                Thor::Definitions::Status reserve(const uint32_t &timeout_ms = 0u);

                /**
				 *  @brief Attempts to release the hardware this SPIClass is tied to
                 *
                 *  This will only work if the class instance holds ownership of the hardware and the previous transfer signals
                 *  that it has completed. It cannot be used to force a release of another instance's lock.
				 *
                 *  @param[in]  timeout_ms  The number of milliseconds to achieve successful release
				 *  @return Thor::Definitions::Status
				 */
    			Thor::Definitions::Status release(const uint32_t &timeout_ms = 0u);

				/**
				 *  @brief Writes a buffer of data
				 *
				 *  @param[in] 	txBuffer 		Input data buffer that will be written to MOSI
				 *  @param[in] 	length 			Number of bytes to be transfered
				 *  @param[in]	autoDisableCS 	Optionally disable the chip select line after the transmition is complete
                 *  @param[in]  timeoutMS       When in blocking mode, this is how long to wait for transfer complete. Interrupt/DMA ignores this value.
				 *  @return Thor::Definitions::Status
				 */
                Thor::Definitions::Status writeBytes(const uint8_t *const txBuffer, size_t length = 0,
                    const bool &autoDisableCS = true, const bool &autoRelease = false, uint32_t timeoutMS = 10);

                /**
				 *  @brief Reads a buffer of data
                 *
                 *  use paragraphs for documenting master vs slave behavior
				 *
				 *  @param[in] 	txBuffer 		Input data buffer that will be written to MOSI
				 *  @param[in] 	length 			Number of bytes to be transfered
				 *  @param[in]	autoDisableCS 	Optionally disable the chip select line after the transmition is complete
                 *  @param[in]  timeoutMS       When in blocking mode, this is how long to wait for transfer complete. Interrupt/DMA ignores this value.
				 *  @return Thor::Definitions::Status
				 */
                Thor::Definitions::Status readBytes(uint8_t *const rxBuffer, size_t length = 0,
                    const bool &autoDisableCS = true, const bool &autoRelease = false, uint32_t timeoutMS = 10);

                /**
				 *  @brief Simultaneously writes and reads data
				 *
				 *  @param[in] 	txBuffer	    Data buffer that will be written to MOSI
				 *  @param[out] rxBuffer 		Data buffer that will have MISO written to it
				 *  @param[in] 	length 			Number of bytes to be transfered
				 *  @param[in] 	autoDisableCS 	Optionally disable the chip select line after the transmition is complete
                 *  @param[in]  timeoutMS       When in blocking mode, this is how long to wait for transfer complete. Interrupt/DMA ignores this value.
				 *  @return Thor::Definitions::Status
				 */
                Thor::Definitions::Status readWriteBytes(const uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length = 0,
                    const bool &autoDisableCS = true, const bool &autoRelease = false, uint32_t timeoutMS = 10);

				/**
				 *  @brief Writes the device slave select line high or low
				 *
				 *  @param[in]	state 	The state to drive the chip select line to
				 *  @return Thor::Definitions::Status
				 */
				Thor::Definitions::Status setChipSelect(const Thor::Definitions::GPIO::LogicLevel &state);

                /**
				 *  @brief Sets the behavior of the chip select line
				 *
				 *  @param[in]	mode    The desired operating mode
				 *  @return Thor::Definitions::Status
				 */
                Thor::Definitions::Status setChipSelectControlMode(const Thor::Definitions::SPI::ChipSelectMode &mode);

                /**
				 *  @brief Attaches an external pin to be used as the slave select
				 *
				 *  @param[in] 	slave_select 	Reference to the initialized GPIO pin
				 *  @return Thor::Definitions::Status
				 */
                Thor::Definitions::Status attachChipSelect(const Thor::Peripheral::GPIO::GPIOClass_sPtr &chipSelect);

                /**
				 *  @brief Removes any previous pin that was attached as the slave select
				 *
				 *  @return Thor::Definitions::Status
				 */
				Thor::Definitions::Status detachChipSelect();

				/**
				 * 	@brief Place the specified peripheral into a given mode
				 *
				 *	@param[in]	periph	Explicitly state which peripheral subsystem (TX or RX) to set
				 *	@param[in] 	mode	The corresponding mode for the peripheral to enter
				 *	@return Thor::Definitions::Status
				 **/
                Thor::Definitions::Status setMode(const Thor::Definitions::SubPeripheral &periph, const Thor::Definitions::Modes &mode);

                /**
				 * 	@brief Get the current mode of a subperipheral
				 *
				 *	@param[in]	periph	The subperipheral to be checked
				 *	@param[out] mode	The current mode this peripheral is in
				 *	@return Thor::Definitions::Status
				 **/
                Thor::Definitions::Status getMode(const Thor::Definitions::SubPeripheral &periph, Thor::Definitions::Modes *const mode);

                /**
				 *  @brief Updates the clock frequency of an already initialized SPI object
				 *
				 *	@param[in] freq		Desired clock frequency in Hz
				 *	@return Thor::Definitions::Status
				 **/
				Thor::Definitions::Status setClockFrequency(const uint32_t &freq);

                /**
				 *  @brief Gets the output clock frequency setting
				 *
				 *	@param[out] freq    The current clock frequency in Hz
				 *	@return Thor::Definitions::Status
				 **/
                Thor::Definitions::Status getClockFrequency(uint32_t *const freq);

				/**
				 *  @brief Normal interrupt based ISR handler
				 *
				 *  @return void
				 */
				void IRQHandler();

				/**
				 *  @brief DMA TX ISR handler
				 *
				 *  @return void
				 */
				void IRQHandler_TXDMA();

				/**
				 *  @brief DMA RX ISR handler
				 *
				 *  @return void
				 */
				void IRQHandler_RXDMA();

                #ifdef USING_FREERTOS
                void attachThreadTrigger(const Thor::Definitions::Interrupt::Trigger &trig, const SemaphoreHandle_t *const semphr);

                void removeThreadTrigger(const SemaphoreHandle_t *const semphr);
                #endif


                SPIClass() = default;
				~SPIClass() = default;

              protected:
                friend void(::HAL_SPI_TxCpltCallback)(SPI_HandleTypeDef *hspi);
                friend void(::HAL_SPI_TxRxCpltCallback)(SPI_HandleTypeDef *hspi);
    			friend void(::HAL_SPI_ErrorCallback)(SPI_HandleTypeDef *hspi);

                static uint32_t getPrescaler(const int &channel, const uint32_t &freq);

                static uint32_t getFrequency(const int &channel, const uint32_t &prescaler);

                Thor::Definitions::Status transfer_blocking(const uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length,
                    const bool &autoDisableCS = true, const bool &autoRelease = false, uint32_t timeoutMS = 10);

                Thor::Definitions::Status transfer_interrupt(const uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length,
                    const bool &autoDisableCS = true, const bool &autoRelease = false);

                Thor::Definitions::Status transfer_dma(const uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length,
                    const bool &autoDisableCS = true, const bool &autoRelease = false);

    			bool isAvailable(const uint32_t &ownerID);

    			bool isLocked();

    			bool isOwner(const uint32_t &ownerID = std::numeric_limits<uint32_t>::max());

    			bool isTransferComplete();

    			bool isExternallyReserved();

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
                SPIClass(const uint8_t& channel);

                bool alternateCS;
    			uint32_t ownerID;
                uint8_t spi_channel;

                Thor::Definitions::SPI::ChipSelectMode chipSelectMode = Thor::Definitions::SPI::ChipSelectMode::MANUAL;
                Thor::Definitions::Modes txMode = Thor::Definitions::Modes::MODE_UNDEFINED;
                Thor::Definitions::Modes rxMode = Thor::Definitions::Modes::MODE_UNDEFINED;

				struct SPIClassStatus
				{
					bool gpio_enabled = false;
					bool spi_enabled = false;
					bool spi_interrupts_enabled = false;
					bool dma_enabled_tx = false;
					bool dma_enabled_rx = false;
					bool dma_interrupts_enabled_tx = false;
					bool dma_interrupts_enabled_rx = false;
				} hardwareStatus;

				SPI_HandleTypeDef spi_handle;
				DMA_HandleTypeDef hdma_spi_tx;
				DMA_HandleTypeDef hdma_spi_rx;

				IT_Initializer ITSettingsHW, ITSettings_DMA_TX, ITSettings_DMA_RX;

				Thor::Peripheral::GPIO::GPIOClass_sPtr externalCS;
                Thor::Peripheral::GPIO::GPIOClass_uPtr MOSI, MISO, SCK, CS;

                void resetISRFlags();
                void lock(const bool &external_lock = false);
    			void unlock();

				void SPI_GPIO_Init();
				void SPI_GPIO_DeInit();

				void SPI_Init();
				void SPI_DeInit();
				void SPI_EnableClock();
				void SPI_DisableClock();
				void SPI_DMA_EnableClock();
				void SPI_EnableInterrupts();
				void SPI_DisableInterrupts();

                void SPI_DMA_Init(const Thor::Definitions::SubPeripheral& periph);
				void SPI_DMA_DeInit(const Thor::Definitions::SubPeripheral& periph);
				void SPI_DMA_EnableInterrupts(const Thor::Definitions::SubPeripheral& periph);
				void SPI_DMA_DisableInterrupts(const Thor::Definitions::SubPeripheral& periph);


			};


            class ChimeraSPI : public Chimera::SPI::Interface
            {
            public:
                ChimeraSPI(const int& channel);

                ~ChimeraSPI() = default;

                Chimera::SPI::Status init(const Chimera::SPI::Setup& setupStruct) final override;

                Chimera::SPI::Status setChipSelect(const Chimera::GPIO::State &value) final override;

                Chimera::SPI::Status setChipSelectControlMode(const Chimera::SPI::ChipSelectMode &mode) final override;

                Chimera::SPI::Status writeBytes(const uint8_t *const txBuffer, size_t length,
                    const bool &disableCS = true, const bool &autoRelease = false, uint32_t timeoutMS = 10) final override;

                Chimera::SPI::Status readBytes(uint8_t *const rxBuffer, size_t length,
                    const bool &disableCS = true, const bool &autoRelease = false, uint32_t timeoutMS = 10) final override;

                Chimera::SPI::Status readWriteBytes(const uint8_t *const txBuffer, uint8_t *const rxBuffer, size_t length,
                    const bool &disableCS = true, const bool &autoRelease = false, uint32_t timeoutMS = 10) final override;

                Chimera::SPI::Status setPeripheralMode(const Chimera::SPI::SubPeripheral &periph, const Chimera::SPI::SubPeripheralMode &mode) final override;

                Chimera::SPI::Status setClockFrequency(const uint32_t &freq) final override;

                Chimera::SPI::Status getClockFrequency(uint32_t *const freq) final override;

                Chimera::SPI::Status reserve(const uint32_t &timeout_ms = 0u) final override;

                Chimera::SPI::Status release(const uint32_t &timeout_ms = 0u) final override;

                Chimera::SPI::Status onWriteCompleteCallback(const Chimera::void_func_void func) final override;

                Chimera::SPI::Status onReadCompleteCallback(const Chimera::void_func_void func) final override;

                Chimera::SPI::Status onReadWriteCompleteCallback(const Chimera::void_func_void func) final override;

                Chimera::SPI::Status onErrorCallback(const Chimera::void_func_uint32_t func) final override;

                #if defined(USING_FREERTOS)
                Chimera::SPI::Status attachEventWakeup(const Chimera::FreeRTOS::SPIEvent &event, const SemaphoreHandle_t *const semphr) final override;

                Chimera::SPI::Status removeEventWakeup(const SemaphoreHandle_t *const semphr) final override;
                #endif

                Thor::Peripheral::SPI::Config convertInitSettings(const Chimera::SPI::Setup &setup);
                static uint32_t convertMode(const Chimera::SPI::Mode &mode, bool *const error);
                static uint32_t convertDataSize(const Chimera::SPI::DataSize &size, bool *const error);
                static uint32_t convertBitOrder(const Chimera::SPI::BitOrder &order, bool *const error);
                static uint32_t convertClockPhase(const Chimera::SPI::ClockMode &mode, bool *const error);
                static uint32_t convertClockPolarity(const Chimera::SPI::ClockMode &mode, bool *const error);
                static uint32_t convertBaudRatePrescaler(const int &channel, const uint32_t &freq, bool *const error);

                static Thor::Definitions::SPI::ChipSelectMode convertChipSelectMode(const Chimera::SPI::ChipSelectMode &mode, bool *const error);

              private:
                bool conversionError;
                int channel;
                SPIClass_sPtr spi;
            };
		}
	}
}

#endif /* SPI_H_*/
