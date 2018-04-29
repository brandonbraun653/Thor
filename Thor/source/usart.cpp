/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/static_vector.hpp>

/* Project Includes */
#include <Thor/include/usart.h>
#include <Thor/include/exceptions.h>
#include <Thor/include/interrupt.h>

using namespace Thor::Definitions::Serial;
using namespace Thor::Definitions::USART;
using namespace Thor::Peripheral::USART;
using namespace Thor::Peripheral::GPIO;
using namespace Thor::Defaults::Serial;

#if defined(USING_FREERTOS)
static boost::container::static_vector<SemaphoreHandle_t, MAX_SERIAL_CHANNELS + 1> usart_semphrs(MAX_SERIAL_CHANNELS + 1);
#endif

static boost::container::static_vector<USARTClass_sPtr, MAX_SERIAL_CHANNELS + 1> usartObjects(MAX_SERIAL_CHANNELS + 1);

static boost::container::flat_map<USART_TypeDef*, uint32_t> usartObjectIndex =
{
	#if defined(USART1)
	{ USART1, 1 },
	#endif
	#if defined(USART2)
	{ USART2, 2 },
	#endif
	#if defined(USART3)
	{ USART3, 3 },
	#endif
	#if defined(USART4)
	{ USART4, 4 },
	#endif
	#if defined(USART5)
	{ USART5, 5 },
	#endif
	#if defined(USART6)
	{ USART6, 6 },
	#endif
	#if defined(USART7)
	{ USART7, 7 },
	#endif
	#if defined(USART8)
	{ USART8, 8 },
	#endif
};

static boost::container::flat_map<USART_TypeDef*, uint32_t> usartClockMask =
{
	//#if defined(STM32F446xx) || defined(STM32F767xx)
	//	#if defined(UART4)
	//	{ UART4, RCC_APB1ENR_UART4EN },
	//	#endif
	//	#if defined(UART5)
	//	{ UART5, RCC_APB1ENR_UART5EN },
	//	#endif
	//	#if defined(UART7)
	//	{ UART7, RCC_APB1ENR_UART7EN },
	//	#endif
	//	#if defined(UART8)
	//	{ UART8, RCC_APB1ENR_UART8EN },
	//	#endif
	//#endif
};

namespace Thor
{
	namespace Peripheral
	{
		namespace USART
		{
			
			Status USARTClass::begin()
			{
				return PERIPH_OK;
			}

			Status USARTClass::begin(const BaudRate& baud)
			{
				return PERIPH_OK;
			}

			Status USARTClass::begin(const BaudRate& baud, const Modes& tx_mode, const Modes& rx_mode)
			{
				return PERIPH_OK;
			}

			Status USARTClass::write(uint8_t* val, size_t length)
			{
				return PERIPH_OK;
			}

			Status USARTClass::write(char* string, size_t length)
			{
				return PERIPH_OK;
			}

			Status USARTClass::write(const char* string)
			{
				return PERIPH_OK;
			}

			Status USARTClass::write(const char* string, size_t length)
			{
				return PERIPH_OK;
			}


			USARTClass::USARTClass(const int& channel)
			{

			}


			boost::shared_ptr<USARTClass> USARTClass::create(const int channel)
			{
				//TODO: Put runtime assertion here and gracefully fail if channel outside bounds

				//Forced to use this instead of make_shared due to required private constructor
				boost::shared_ptr<USARTClass> newClass(new USARTClass(channel));

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