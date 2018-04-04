#include <Thor/include/gpio.h>


namespace Thor
{
	namespace Peripheral
	{
		namespace GPIO
		{
			using namespace Thor::Definitions::GPIO;

			/*----------------------------------
			* Constructors
			*---------------------------------*/
			GPIOClass::GPIOClass() {}

			GPIOClass::GPIOClass(GPIO_TypeDef *GPIOx, 
				GPIO_PinNum_TypeDef PIN_x, 
				GPIO_Speed_TypeDef SPEED,
				uint32_t ALTERNATE)
			{
				/*--------------------------------------
				* Store Thor style config defaults
				*-------------------------------------*/
				pinConfig.GPIOx		= GPIOx;
				pinConfig.pinNum	= PIN_x;
				pinConfig.speed		= SPEED;
				pinConfig.alternate = ALTERNATE;
	
				/*--------------------------------------
				* Initialize the pin
				*-------------------------------------*/
				InitStruct.Pin			= (uint32_t)pinConfig.pinNum;
				InitStruct.Speed		= (uint32_t)pinConfig.speed;
				InitStruct.Mode			= GPIO_MODE_INPUT;
				InitStruct.Pull			= GPIO_NOPULL;
				InitStruct.Alternate	= ALTERNATE;
			}


			/*----------------------------------
			* Basic User Functions
			*---------------------------------*/
			void GPIOClass::mode(GPIO_Mode_TypeDef Mode, GPIO_Pull_TypeDef Pull)
			{
				// TODO: Pullup/pulldown doesn't actually work in PP mode
				pinConfig.mode = Mode;
				pinConfig.pull = Pull;
	
				InitStruct.Mode = (uint32_t)Mode;
				InitStruct.Pull = (uint32_t)Pull;
				GPIO_Init(pinConfig.GPIOx, &InitStruct);
			}

			void GPIOClass::write(LogicLevel state)
			{
				switch (state)
				{
				case LOW:
				case OFF:
				case ZERO:
					HAL_GPIO_WritePin(pinConfig.GPIOx,
						(uint16_t)pinConfig.pinNum,
						GPIO_PIN_RESET);
					break;
	
				case HIGH:
				case ON:
				case ONE:
					HAL_GPIO_WritePin(pinConfig.GPIOx,
						(uint16_t)pinConfig.pinNum,
						GPIO_PIN_SET);
					break;
				}
			}

			void GPIOClass::read(bool *state)
			{
				switch (HAL_GPIO_ReadPin(pinConfig.GPIOx, (uint16_t)pinConfig.pinNum))
				{
				case GPIO_PIN_SET:
					*state = true;
					break;
	
				case GPIO_PIN_RESET:
					*state = false;
					break;
				}
			}

			void GPIOClass::analogRead(int *data)
			{
	
			}

			void GPIOClass::toggle()
			{
				HAL_GPIO_TogglePin(pinConfig.GPIOx, (uint16_t)pinConfig.pinNum);
			}


			/*----------------------------------
			* Advanced User Functions
			*---------------------------------*/
			void GPIOClass::fast_write(LogicLevel state)
			{
	
			}

			void GPIOClass::fast_toggle()
			{
	
			}


			void GPIOClass::attachIT(void(*)(void))
			{
	
			}

			void GPIOClass::reconfigure(GPIO_TypeDef *GPIOx, 
				GPIO_PinNum_TypeDef PIN_x, 
				GPIO_Speed_TypeDef SPEED,
				uint32_t ALTERNATE)
			{
				/*--------------------------------------
				* Store Thor style config defaults
				*-------------------------------------*/
				pinConfig.GPIOx		= GPIOx;
				pinConfig.pinNum	= PIN_x;
				pinConfig.speed		= SPEED;
				pinConfig.alternate = ALTERNATE;
	
	
				/*--------------------------------------
				* Initialize the pin
				*-------------------------------------*/
				InitStruct.Pin			= (uint32_t)pinConfig.pinNum;
				InitStruct.Speed		= (uint32_t)pinConfig.speed;
				InitStruct.Mode			= GPIO_MODE_INPUT;
				InitStruct.Pull			= GPIO_NOPULL;
				InitStruct.Alternate	= ALTERNATE;
	
	
				GPIO_Init(pinConfig.GPIOx, &InitStruct);
			}
			
			/************************************************************************/
			/* EXPORTED FUNCTIONS                                                   */
			/************************************************************************/

			/** \brief Initializes a GPIO port to a desired config

			*/
			void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *InitStruct)
			{	
				//Start the clock if it hasn't been already
				GPIO_ClockEnable(GPIOx);
	
				//Apply the configuration
				HAL_GPIO_Init(GPIOx, InitStruct);
			}

			/** \brief Initializes a GPIO port clock

			*/
			void GPIO_ClockEnable(GPIO_TypeDef *GPIOx)
			{
				#if defined (STM32F767xx) || defined(STM32F446xx)
				if (GPIOx == GPIOA)
					__GPIOA_CLK_ENABLE();
	
				if (GPIOx == GPIOB)
					__GPIOB_CLK_ENABLE();
	
				if (GPIOx == GPIOC)
					__GPIOC_CLK_ENABLE();
	
				if (GPIOx == GPIOD)
					__GPIOD_CLK_ENABLE();
	
				if (GPIOx == GPIOE)
					__GPIOE_CLK_ENABLE();
	
				if (GPIOx == GPIOF)
					__GPIOF_CLK_ENABLE();
	
				if (GPIOx == GPIOG)
					__GPIOG_CLK_ENABLE();
	
				if (GPIOx == GPIOH)
					__GPIOH_CLK_ENABLE();
				#endif
	
				#if defined (STM32F767xx)
				if (GPIOx == GPIOI)
					__GPIOI_CLK_ENABLE();
	
				if (GPIOx == GPIOJ)
					__GPIOJ_CLK_ENABLE();
	
				if (GPIOx == GPIOK)
					__GPIOK_CLK_ENABLE();
				#endif
			}
			
		}
	}
}
	
