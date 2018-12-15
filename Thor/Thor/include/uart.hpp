/** @example uart_basic_example.cpp
 *	This demonstrates a simple way to get an instance of Thor::Peripheral::UART::UARTClass up and running quickly. */

#pragma once
#ifndef UART_H_
#define UART_H_

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/container/static_vector.hpp>
#pragma GCC diagnostic pop

/* Thor Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>
#include <Thor/include/defaults.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/interrupt.hpp>

#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include <Thor/include/exti.hpp>
#endif

/** @namespace Thor */
namespace Thor
{
	/** @namespace Thor::Peripheral */
	namespace Peripheral
	{
		/** @namespace Thor::Peripheral::UART */
		namespace UART
		{

			/** A higher level uart interface built ontop of the STM32 HAL that abstracts away most
			 *	of the details associated with setup and general usage. It supports both transmission\n
			 *	and reception in 3 modes [blocking, interrupt, dma] and does not require that TX and RX
			 *	share the same mode for proper operation. In addition, data is copied to an internal buffer\n
			 *	for all non-blocking transmissions (IT/DMA) so that the user doesn't have to worry about
			 *	destroyed or mutable data.
			 *
			 *	@note If using FreeRTOS, the class is threadsafe and allows multiple sources to write and
			 *		  read on a class object up to an internal buffer limit defined by Thor::Definitions::Serial::UART_BUFFER_SIZE
			 **/
            class UARTClass : public Thor::Definitions::Serial::SerialInterface
			{
			public:
				/**
                *   Initializes with a given baud rate and TX/RX modes.
                *
				*	@param[in]  baud	    Desired baud rate. Accepts standard rates from Thor::Definitions::Serial::BaudRate
				*	@param[in]  tx_mode	    Sets the TX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				*	@param[in]  rx_mode	    Sets the RX mode to Blocking, Interrupt, or DMA from Thor::Definitions::Serial::Modes
				*	@return	Thor::Definitions::Status
				*/
                Thor::Definitions::Status begin(const Thor::Definitions::Serial::BaudRate baud = Thor::Definitions::Serial::BaudRate::SERIAL_BAUD_115200,
					                            const Thor::Definitions::Modes tx_mode = Thor::Definitions::Modes::BLOCKING,
					                            const Thor::Definitions::Modes rx_mode = Thor::Definitions::Modes::BLOCKING) override;

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
				*   Sets a new baud rate
                *
                *   @param[in]  baud        Buad rate to be set
                *   @return Thor::Definitions::Status
				*/
				Thor::Definitions::Status setBaud(const Thor::Definitions::Serial::BaudRate baud) override;

				/**
                *   Writes data to the serial output gpio
                *
				*	@param[in]  val		    Pointer to an array of data to be sent out
				*	@param[in]  length	    The length of data to be sent out
				*	@return	Thor::Definitions::Status
				*/
				Thor::Definitions::Status write(const uint8_t *const val, const size_t length) override;

				/**
                *   Writes data to the serial output gpio
				*
                *   @param[in]  string	    Pointer to a mutable character array
				*	@param[in]  length	    The length of data to be sent out
				*	@return	Thor::Definitions::Status
				*/
				Thor::Definitions::Status write(char *const string, const size_t length) override;

				/**
                *   Writes data to the serial output gpio
                *
				*	@param[in]  string	    Pointer to a immutable character array. The length is internally calculated with strlen()
				*	@return	Thor::Definitions::Status
				*/
				Thor::Definitions::Status write(const char *const string) override;

				/**
                *   Writes data to the serial output gpio
                *
				*	@param[in]  string	    Pointer to an immutable character array
				*	@param[in]  length	    The length of data to be sent out
				*	@return	Thor::Definitions::Status
				*/
				Thor::Definitions::Status write(const char *const string, const size_t length) override;

				/**
                *   Commands the RX peripheral to read a single transmission of known length into the provided buffer.
				*
                *   @param[out] buff	    An external buffer to write the received data to
				*	@param[in]  length	    The number of bytes to be received
				*
				*	@note Only use this for receptions that have a fixed, known length. For transmissions that last longer than
				*		  the given 'length' value, it will simply be ignored and lost forever. Poor data.
				*/
				Thor::Definitions::Status read(uint8_t *const buff, size_t length) override;



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
				void attachThreadTrigger(const Thor::Definitions::Interrupt::Trigger trig, SemaphoreHandle_t *const  semphr) override;

				/**
                *   Removes a trigger source
				*
                *   @param[in]  trig	    The source to be removed, of type Thor::Definitions::Interrupt::Trigger
				*/
				void removeThreadTrigger(const Thor::Definitions::Interrupt::Trigger trig) override;
				#endif


			private:
				UARTClass(const uint8_t channel, const Thor::Definitions::Serial::SerialPins &pinConfig);

			public:
				/** A factory method to create a new UARTClass object.
				 *
				 *	This method intentionally replaces the typical constructor due to the need to register
				 *	the shared_ptr with a static_vector that allows runtime deduction of which class to call
				 *	inside of an ISR. This is done for simplicity.
				 *
				 *	@param[in] channel Hardware peripheral channel number (i.e. 1 for UART1, 4 for UART4, etc)
				 *	@return Shared pointer to the new object
				 **/
				static std::shared_ptr<UARTClass> create(const uint8_t channel, const Thor::Definitions::Serial::SerialPins &pinConfig);
				~UARTClass();

				/** Easily references buffered data for TX or RX */
				struct UARTPacket
				{
					uint8_t* data_ptr = nullptr;	/**< Contains the buffer address where data is stored */
					uint16_t bytesRead = 0;			/**< Number of bytes already read from the packet (currently used in eRPC calls) */
					size_t length = 0;				/**< Number of bytes contained in data_ptr */
				};


			private:
                friend void(::HAL_UART_TxCpltCallback)(UART_HandleTypeDef *UsartHandle);
    			friend void(::HAL_UART_RxCpltCallback)(UART_HandleTypeDef *UsartHandle);
                friend void(::UART1_IRQHandler)(void);
                friend void(::UART2_IRQHandler)(void);
                friend void(::UART3_IRQHandler)(void);
                friend void(::UART4_IRQHandler)(void);
                friend void(::UART5_IRQHandler)(void);
                friend void(::UART6_IRQHandler)(void);
                friend void(::UART7_IRQHandler)(void);
                friend void(::UART8_IRQHandler)(void);


				int uart_channel;																/**< Numerical representation of the UART instance, zero is invalid */
				bool tx_complete = true;														/**< Indicates if a transmission has been completed */
				bool rx_complete = true;														/**< Indicates if a reception has been completed */
				bool RX_ASYNC = true;															/**< Enables/Disables asynchronous reception of data */
                Thor::Definitions::Modes txMode;										/**< Logs which mode the TX peripheral is currently in */
                Thor::Definitions::Modes rxMode;										/**< Logs which mode the RX peripheral is currently in */

				struct UARTClassStatus
				{
					bool gpio_enabled = false;
					bool uart_enabled = false;
					bool dma_enabled_tx = false;
					bool dma_enabled_rx = false;
					bool uart_interrupts_enabled = false;
					bool dma_interrupts_enabled_tx = false;
					bool dma_interrupts_enabled_rx = false;
				} UART_PeriphState;															/**< Flags that allow more precise configuring of the low level hardware during init/de-init */

				/*-------------------------------
				* Object Pointers / Handles
				*------------------------------*/
				UART_HandleTypeDef uart_handle;
				DMA_HandleTypeDef hdma_uart_tx;
				DMA_HandleTypeDef hdma_uart_rx;
				Thor::Peripheral::GPIO::GPIOClass_sPtr tx_pin;
				Thor::Peripheral::GPIO::GPIOClass_sPtr rx_pin;

				/* Local copy of interrupt settings */
				IT_Initializer ITSettings_HW, ITSettings_DMA_TX, ITSettings_DMA_RX;

				/*-------------------------------
				* Low Level Setup/Teardown Functions
				*------------------------------*/
                void IRQHandler();
				void IRQHandler_TXDMA();
				void IRQHandler_RXDMA();

				void UART_GPIO_Init();
				void UART_GPIO_DeInit();

				void UART_Init();
				void UART_DeInit();
				void UART_EnableClock();
				void UART_DisableClock();
				void UART_DMA_EnableClock();
				void UART_EnableInterrupts();
				void UART_DisableInterrupts();

                void UART_DMA_Init(const Thor::Definitions::SubPeripheral& periph);
                void UART_DMA_DeInit(const Thor::Definitions::SubPeripheral& periph);
                void UART_DMA_EnableIT(const Thor::Definitions::SubPeripheral& periph);
                void UART_DMA_DisableIT(const Thor::Definitions::SubPeripheral& periph);

				/*-------------------------------
				* Error Handler Functions
				*------------------------------*/
				void UART_OverrunHandler();
			};

			/** The most common way of referencing an instance of UARTClass. It is intended that future
			 *	libraries will want to pass around copies of a class and a shared_ptr was chosen for ease\n
			 *	of use and safe destruction. */
			typedef boost::shared_ptr<UARTClass> UARTClass_sPtr;
		}


		/*-------------------------------
		* Utility functions
		*------------------------------*/
		extern void UART_EnableIT_IDLE(UART_HandleTypeDef *UartHandle);
		extern void UART_DisableIT_IDLE(UART_HandleTypeDef *UartHandle);
		extern void UART_ClearIT_IDLE(UART_HandleTypeDef *UartHandle);
	}
}
#endif /* !UART_H_ */
