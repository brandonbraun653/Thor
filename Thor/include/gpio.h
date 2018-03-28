
#ifndef GPIO_H_
#define GPIO_H_

/************************************************************************/
/*							   Includes                                 */
/************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "thor_config.h"
#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#endif
#include "thor_definitions.h"

#include "../libraries/Boost/boost/shared_ptr.hpp"
/************************************************************************/
/*								Defines                                 */
/************************************************************************/
#define NOALTERNATE (0x08000CC8)	//Discovered from debug mode

/************************************************************************/
/*								 Types                                  */
/************************************************************************/
typedef enum
{
	NOPIN	= -1,
	PIN_0	= GPIO_PIN_0,
	PIN_1	= GPIO_PIN_1,
	PIN_2	= GPIO_PIN_2,
	PIN_3	= GPIO_PIN_3,
	PIN_4	= GPIO_PIN_4,
	PIN_5	= GPIO_PIN_5,
	PIN_6	= GPIO_PIN_6,
	PIN_7	= GPIO_PIN_7,
	PIN_8	= GPIO_PIN_8,
	PIN_9	= GPIO_PIN_9,
	PIN_10	= GPIO_PIN_10,
	PIN_11	= GPIO_PIN_11,
	PIN_12	= GPIO_PIN_12,
	PIN_13	= GPIO_PIN_13,
	PIN_14	= GPIO_PIN_14,
	PIN_15	= GPIO_PIN_15,
	PIN_ALL	= GPIO_PIN_All
} GPIO_PinNum_TypeDef;


typedef enum
{
	NOMODE				= -1,
	INPUT				= GPIO_MODE_INPUT,
	OUTPUT_PP			= GPIO_MODE_OUTPUT_PP,
	OUTPUT_OD			= GPIO_MODE_OUTPUT_OD,
	ALT_PP				= GPIO_MODE_AF_PP,
	ALT_OD				= GPIO_MODE_AF_OD,
	ANALOG				= GPIO_MODE_ANALOG,
	IT_RISING			= GPIO_MODE_IT_RISING,
	IT_FALLING			= GPIO_MODE_IT_FALLING,
	IT_RISING_FALLING	= GPIO_MODE_IT_RISING_FALLING,
	EVT_RISING			= GPIO_MODE_EVT_RISING,
	EVT_FALLING			= GPIO_MODE_EVT_FALLING,
	EVT_RISING_FALLING	= GPIO_MODE_EVT_RISING_FALLING
} GPIO_Mode_TypeDef;


typedef enum
{
	NOSPD		= -1,
	LOW_SPD		= GPIO_SPEED_FREQ_LOW,
	MEDIUM_SPD	= GPIO_SPEED_FREQ_MEDIUM,
	HIGH_SPD	= GPIO_SPEED_FREQ_HIGH,
	ULTRA_SPD	= GPIO_SPEED_FREQ_VERY_HIGH
} GPIO_Speed_TypeDef;


typedef enum
{	
	NOPULL = GPIO_NOPULL,
	PULLUP = GPIO_PULLUP,
	PULLDN = GPIO_PULLDOWN
} GPIO_Pull_TypeDef;


typedef struct
{
	GPIO_TypeDef		*GPIOx		= GPIOA;
	GPIO_Speed_TypeDef	speed		= HIGH_SPD;
	GPIO_Mode_TypeDef	mode		= INPUT;
	GPIO_PinNum_TypeDef	pinNum		= PIN_0;
	GPIO_Pull_TypeDef   pull		= NOPULL;
	uint32_t			alternate	= NOALTERNATE;
} GPIO_PinConfig_TypeDef;

/************************************************************************/
/*							Exported Variables                          */
/************************************************************************/


/************************************************************************/
/*							Exported Functions                          */
/************************************************************************/
extern void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *InitStruct);
extern void GPIO_ClockEnable(GPIO_TypeDef *GPIOx);

/************************************************************************/
/*								Classes                                 */
/************************************************************************/
class GPIOClass;
typedef boost::shared_ptr<GPIOClass> GPIOClass_sPtr;

class GPIOClass
{
public:
	/*----------------------------------
	* Basic User Functions
	*---------------------------------*/
	void mode(GPIO_Mode_TypeDef Mode, GPIO_Pull_TypeDef Pull=NOPULL);
	void write(ThorDef::GPIO::LogicLevel state);
	void read(bool *state);
	void analogRead(int *data);
	void toggle();
	
	/*----------------------------------
	* Advanced User Functions
	*---------------------------------*/
	void fast_write(ThorDef::GPIO::LogicLevel state);
	void fast_toggle();
	void attachIT(void(*)(void));
	void reconfigure(GPIO_TypeDef *GPIOx, GPIO_PinNum_TypeDef PIN_x, GPIO_Speed_TypeDef SPEED, uint32_t ALTERNATE);
	
	/*----------------------------------
	* Constructors
	*---------------------------------*/
	GPIOClass(void);
	GPIOClass(GPIO_TypeDef *GPIOx,
			GPIO_PinNum_TypeDef PIN_x,
			GPIO_Speed_TypeDef SPEED,
		    uint32_t ALTERNATE);
	
private:
	GPIO_InitTypeDef InitStruct;
	GPIO_PinConfig_TypeDef pinConfig;
};


#endif // !_GPIO_H_
