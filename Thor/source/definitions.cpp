#include <Thor/include/definitions.h>


namespace Thor
{
	namespace Definitions
	{
		namespace GPIO
		{
			boost::container::flat_map<GPIO_TypeDef*, uint32_t> rcc_gpio_mask =
			{ 
				//STM32F7 PDF: RM0410 pg.181
				//STM32F4 PDF: RM0390 pg.143
				#if defined(TARGET_STM32F7) || defined(TARGET_STM32F4)
				{GPIOA, (1u << 0)}, 
				{GPIOB, (1u << 1)}, 
				{GPIOC, (1u << 2)}, 
				{GPIOD, (1u << 3)},
				{GPIOE, (1u << 4)}, 
				{GPIOF, (1u << 5)}, 
				{GPIOG, (1u << 6)}, 
				{GPIOH, (1u << 7)},
				#endif				#if defined(TARGET_STM32F7)				{GPIOI, (1u << 8)}, 				{GPIOJ, (1u << 9)}, 				{GPIOK, (1u << 10)}				#endif
			};
		}
	}
}