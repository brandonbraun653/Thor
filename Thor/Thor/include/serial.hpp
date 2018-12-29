#pragma once
#ifndef SERIAL_H_
#define SERIAL_H_

/* C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <memory>

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/uart.hpp>
#include <Thor/include/usart.hpp>
#include <Thor/include/threads.hpp>

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Peripheral */
	namespace Peripheral
	{
		/** @namespace Thor::Peripheral::Serial */
		namespace Serial
		{
            class SerialClass;

            typedef std::shared_ptr<SerialClass> SerialClass_sPtr;
			typedef std::unique_ptr<SerialClass> SerialClass_uPtr;

            /**
            *   A high level, basic serial interface for rapid prototyping. This is essentially a wrapper over the existing
			*	Thor::Peripheral::UART::UARTClass and Thor::Peripheral::USART::USARTClass interfaces. The goal is to abstract
            *   away from the specific UART and USART peripherals so that the user only has to call 'SerialClass(1)' or 'SerialClass(x)'
            *   to generate a serial object quickly without having to worry about the underlying hardware type.
			*/
			class SerialClass : public Thor::Threading::Lockable, public Thor::Definitions::Serial::SerialInterface
			{
			public:
                SerialClass(const uint8_t channel);
                SerialClass(const uint8_t channel, const Thor::Definitions::Serial::SerialPins &pinConfig);
                ~SerialClass() = default;

				/**
                *   Initializes with a given baud rate and TX/RX modes.
                *
				*	@param[in]  baud	    Desired baud rate. Accepts standard rates from Thor::Definitions::Serial::BaudRate
				*	@param[in]  tx_mode	    Sets the TX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				*	@param[in]  rx_mode	    Sets the RX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				*	@return	Thor::Definitions::Status
				*/
                Thor::Definitions::Status begin(const uint32_t baud,
                    const Thor::Definitions::Modes tx_mode,
                    const Thor::Definitions::Modes rx_mode) override;

				/**
                *   Places the specified peripheral into a given mode
				*
                *   @param[in]  periph	    Explicitly states which peripheral subsystem (TX or RX) to set from Thor::Peripheral::Serial::SubPeripheral
				*	@param[in]  mode	    The corresponding mode for the peripheral to enter, from Thor::Peripheral::Serial::Modes
				*	@return	Thor::Definitions::Status
				*
				*	@note When setting the RX peripheral to IT or DMA mode, it automatically enables asynchronous data reception
				*/
                Thor::Definitions::Status setMode(const Thor::Definitions::SubPeripheral periph, const Thor::Definitions::Modes mode) override;

				/**
				*   Sets a new baud rate
                *
                *   @param[in]  baud        Buad rate to be set
                *   @return Thor::Definitions::Status
				*/
				Thor::Definitions::Status setBaud(const uint32_t baud) override;

				/**
                *   Writes data to the serial output gpio
                *
				*	@param[in]  val		    Pointer to an array of data to be sent out
				*	@param[in]  length	    The length of data to be sent out
				*	@return	Thor::Definitions::Status
				*/
				Thor::Definitions::Status write(const uint8_t *const val, const size_t length) override;

				/**
                *   Commands the RX peripheral to read a single transmission of known length into the provided buffer.
				*
                *   @param[out] buff	    An external buffer to write the received data to
				*	@param[in]  length	    The number of bytes to be received
				*
				*	@note Only use this for receptions that have a fixed, known length. For transmissions that last longer than
				*		  the given 'length' value, it will simply be ignored and lost forever. Poor data.
				*/
				Thor::Definitions::Status read(uint8_t *const buffer, const size_t length) override;



				void end() override;



				#if defined(USING_FREERTOS)
				/**
                *   Attaches a semaphore to a specific trigger source. When an event is triggered on that source,
				*	the semaphore will be 'given' to and any task waiting on that semaphore will become unblocked.
                *
				*	@param[in]  trig		The source to be triggered on, of type Thor::Definitions::Interrupt::Trigger
				*	@param[in]  semphr	    The address of the semaphore that will be 'given' to upon triggering
                *   @return void
				**/
				void attachThreadTrigger(const Thor::Definitions::Interrupt::Trigger trig, SemaphoreHandle_t *const semphr) override;

				/**
                *   Removes a trigger source
				*
                *   @param[in]  trig	    The source to be removed, of type Thor::Definitions::Interrupt::Trigger
				*/
				void removeThreadTrigger(const Thor::Definitions::Interrupt::Trigger trig) override;
				#endif

                bool reserve(const uint32_t timeout_mS) override;

                bool release(const uint32_t timeout_mS) override;


			private:
				uint8_t serialChannel = 0;
				std::shared_ptr<Thor::Definitions::Serial::SerialInterface> serialObject;
			};


            /**
            *   For documentation, see the Chimera::Serial::Interface class
            */
            #if defined(USING_CHIMERA)
            class ChimeraSerial : public Chimera::Serial::Interface
            {
            public:

                ChimeraSerial(const uint8_t channel);
                ~ChimeraSerial() = default;

                Chimera::Serial::Status begin(const uint32_t baud,
                    const Chimera::Serial::Modes txMode, const Chimera::Serial::Modes rxMode) final override;

                Chimera::Serial::Status setBaud(const uint32_t baud) final override;

                Chimera::Serial::Status setMode(const Chimera::Serial::SubPeripheral periph, const Chimera::Serial::Modes mode) final override;

                Chimera::Serial::Status write(const uint8_t *const buffer, const size_t length) final override;

                Chimera::Serial::Status read(uint8_t *const buffer, const size_t length) final override;

                Chimera::Serial::Status readAsync(uint8_t *const buffer, const size_t maxLen) final override;

                Chimera::Serial::Status enableDoubleBuffering(const Chimera::Serial::SubPeripheral periph,
                    volatile uint8_t *const bufferOne,
                    volatile uint8_t *const bufferTwo,
                    const size_t length) final override;

                Chimera::Serial::Status disableDoubleBuffering() final override;

                Chimera::Serial::Status attachEventNotifier(const Chimera::Serial::Event event, volatile bool *const notifier) final override;

                Chimera::Serial::Status removeEventNotifier(const Chimera::Serial::Event event, volatile bool *const notifier) final override;

                #if defined(USING_FREERTOS)
                Chimera::Serial::Status attachEventNotifier(const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr) final override;

                Chimera::Serial::Status removeEventNotifier(const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr) final override;
                #endif

                void status(Chimera::Serial::HardwareStatus &status) final override;

                bool available(size_t *const bytes = nullptr) final override;

                bool reserve(const uint32_t timeout_mS) final override;

                bool release(const uint32_t timeout_mS) final override;

            private:
                ChimeraSerial() = default;

                SerialClass_sPtr serial;
            };

            #endif /* !USING_CHIMERA */
		}
	}
}




#endif /* !SERIAL_H_ */
