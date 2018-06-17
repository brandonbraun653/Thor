/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

/* Project Includes */
#include <Thor/include/config.hpp>
#include <Thor/include/usart.hpp>
#include <Thor/include/exceptions.hpp>
#include <Thor/include/interrupt.hpp>

#if defined(USING_FREERTOS)
#include <Thor/include/exti.hpp>
static SemaphoreHandle_t usartSemphrs[Thor::Definitions::Serial::MAX_SERIAL_CHANNELS + 1];
TaskTrigger usartTaskTrigger;
#endif


using namespace Thor::Definitions;
using namespace Thor::Definitions::Serial;
using namespace Thor::Definitions::USART;
using namespace Thor::Definitions::Interrupt;
using namespace Thor::Peripheral::USART;
using namespace Thor::Peripheral::GPIO;
using namespace Thor::Defaults::Serial;


static USARTClass_sPtr usartObjects[MAX_SERIAL_CHANNELS + 1];

static const USARTClass_sPtr& getUSARTClassRef(USART_TypeDef* instance)
{
	/* Simply converts the pointer into the raw numerical address value, which be compared against
	the peripheral base address. USARTx is simply (USART_TypeDef*)USARTx_Base. */
	auto i = reinterpret_cast<std::uintptr_t>(instance);
	switch (i)
	{
		#if defined(USART1)
	case USART1_BASE:
		return usartObjects[1];
		break;
		#endif
		#if defined(USART2)
	case USART2_BASE:
		return usartObjects[2];
		break;
		#endif
		#if defined(USART3)
	case USART3_BASE:
		return usartObjects[3];
		break;
		#endif
		#if defined(USART4)
	case USART4_BASE:
		return usartObjects[4];
		break;
		#endif
		#if defined(USART5)
	case USART5_BASE:
		return usartObjects[5];
		break;
		#endif
		#if defined(USART6)
	case USART6_BASE:
		return usartObjects[6];
		break;
		#endif
		#if defined(USART7)
	case USART7_BASE:
		return usartObjects[7];
		break;
		#endif
		#if defined(USART8)
	case USART8_BASE:
		return usartObjects[8];
		break;
		#endif
	};
};

static uint32_t usartClockMask(USART_TypeDef* instance)
{
	auto i = reinterpret_cast<std::uintptr_t>(instance);
	switch (i)
	{
		#if defined(STM32F446xx) || defined(STM32F767xx)
		#if defined(USART1)
	case USART1_BASE:
		return RCC_APB2ENR_USART1EN;
		break;
		#endif
		#if defined(USART2)
	case USART2_BASE:
		return RCC_APB1ENR_USART2EN;
		break;
		#endif
		#if defined(USART3)
	case USART3_BASE:
		return RCC_APB1ENR_USART3EN;
		break;
		#endif
		#if defined(USART6)
	case USART6_BASE:
		return RCC_APB2ENR_USART6EN;
		break;
		#endif
		#endif /* !STM32F446xx  !STM32F767xx */
	};
};


namespace Thor
{
	namespace Peripheral
	{
		namespace USART
		{
			Status USARTClass::begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode)
			{
				return Status::PERIPH_OK;
			}

			Status USARTClass::setMode(const SubPeripheral& periph, const Modes& mode)
			{
				return Status::PERIPH_OK;
			}

			Status USARTClass::write(uint8_t* val, size_t length)
			{
				return Status::PERIPH_OK;
			}

			Status USARTClass::write(char* string, size_t length)
			{
				return Status::PERIPH_OK;
			}

			Status USARTClass::write(const char* string)
			{
				return Status::PERIPH_OK;
			}

			Status USARTClass::write(const char* string, size_t length)
			{
				return Status::PERIPH_OK;
			}

			Status USARTClass::readPacket(uint8_t* buff, size_t buff_length)
			{
				return Status::PERIPH_OK;
			}

			Status USARTClass::readSync(uint8_t* buff, size_t length)
			{
				return Status::PERIPH_OK;
			}

			uint32_t USARTClass::availablePackets()
			{
				return 0;
			}

			size_t USARTClass::nextPacketSize()
			{
				return (size_t)0;
			}

			void USARTClass::end()
			{
			}

			#if defined(USING_FREERTOS)
			void USARTClass::attachThreadTrigger(Trigger trig, SemaphoreHandle_t* semphr)
			{
				//uartTaskTrigger.attachEventConsumer(trig, semphr);
			}

			void USARTClass::removeThreadTrigger(Trigger trig)
			{
				//uartTaskTrigger.removeEventConsumer(trig);
			}

			#endif

			USARTClass::USARTClass(const int& channel, SerialPins* pinConfig)
			{
				usartChannel = channel;

				/* Get the default handle settings for this instance */
				usartHandle.Init = Defaults::Serial::dflt_USART_Init;
				usartHandle.Instance = srl_cfg[usartChannel].instance;

				#if defined(STM32F7)
				//usartHandle.AdvancedInit = Defaults::Serial::
				#endif

				/* Get the default Interrupt settings for this instance */
				ITSettings_HW = srl_cfg[usartChannel].IT_HW;
				ITSettings_DMA_TX = srl_cfg[usartChannel].dmaIT_TX;
				ITSettings_DMA_RX = srl_cfg[usartChannel].dmaIT_RX;

				/* Set up the GPIO pins */
				if (pinConfig)
				{
					tx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
						pinConfig->TX_GPIOx,
						pinConfig->TX_Pin,
						PinSpeed::ULTRA_SPD,
						pinConfig->TX_AltFuncCode);

					rx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
						pinConfig->RX_GPIOx,
						pinConfig->RX_Pin,
						PinSpeed::ULTRA_SPD,
						pinConfig->RX_AltFuncCode);
				}
				else
				{
					tx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
						srl_cfg[usartChannel].txPin.GPIOx,
						srl_cfg[usartChannel].txPin.PinNum,
						srl_cfg[usartChannel].txPin.Speed,
						srl_cfg[usartChannel].txPin.Alternate);

					rx_pin = boost::make_shared<Thor::Peripheral::GPIO::GPIOClass>(
						srl_cfg[usartChannel].rxPin.GPIOx,
						srl_cfg[usartChannel].rxPin.PinNum,
						srl_cfg[usartChannel].rxPin.Speed,
						srl_cfg[usartChannel].rxPin.Alternate);
				}

				/* Initialize the buffer memory */
				TXPacketBuffer.set_capacity(USART_QUEUE_SIZE);	TXPacketBuffer.clear();
				RXPacketBuffer.set_capacity(USART_QUEUE_SIZE);	RXPacketBuffer.clear();

			}

			boost::shared_ptr<USARTClass> USARTClass::create(const int channel, SerialPins* pinConfig)
			{
				//TODO: Put runtime assertion here and gracefully fail if channel outside bounds

				//Forced to use this instead of make_shared due to required private constructor
				boost::shared_ptr<USARTClass> newClass(new USARTClass(channel, pinConfig));

				//Register the object so interrupt handlers know the correct reference
				usartObjects[channel] = newClass;

				return newClass;
			}

			USARTClass::~USARTClass()
			{
			}
		}
	}
}