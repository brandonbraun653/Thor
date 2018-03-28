#include "../include/usart.h"

using namespace Defaults::Serial;

/************************************************************************/
/*							  Constructors                              */
/************************************************************************/
USARTClass::USARTClass()
{
	//Nothing here. This constructor is used for instantiations to 
	//class pointers.
}

USARTClass::USARTClass(uint32_t channel, GPIOClass *txPin, GPIOClass *rxPin)
{
	usart_channel = channel;
	tx_pin = txPin;
	rx_pin = rxPin;
}

/************************************************************************/
/*						    Interrupt Handlers                           */
/************************************************************************/
void USARTClass::IrqHandler(void)
{
	
}

/************************************************************************/
/*						     Public Functions                           */
/************************************************************************/
void USARTClass::begin()
{
// 	/*-------------------------------
// 	* Configure Pin Init
// 	*------------------------------*/
// 	tx_pin->reconfigure(srl_cfg[usart_channel].ioConfig.txPinGPIOx,
// 		srl_cfg[usart_channel].ioConfig.txPinNum,
// 		srl_cfg[usart_channel].ioConfig.txPinSpeed,
// 		srl_cfg[usart_channel].ioConfig.txPinAlternate);
// 	
// 	rx_pin->reconfigure(srl_cfg[usart_channel].ioConfig.rxPinGPIOx,
// 		srl_cfg[usart_channel].ioConfig.rxPinNum,
// 		srl_cfg[usart_channel].ioConfig.rxPinSpeed,
// 		srl_cfg[usart_channel].ioConfig.rxPinAlternate);
// 	
// 	/*-------------------------------
// 	* Configure Pin Mode
// 	*------------------------------*/
// 	tx_pin->mode(srl_cfg[usart_channel].ioConfig.txPinMode,
// 		srl_cfg[usart_channel].ioConfig.txPinPull);
// 	
// 	rx_pin->mode(srl_cfg[usart_channel].ioConfig.rxPinMode,
// 		srl_cfg[usart_channel].ioConfig.txPinPull);
}


void USARTClass::write(uint32_t val, uint32_t length)
{
	
}

void USARTClass::read(uint32_t lenght)
{
	
}

void USARTClass::flush()
{
	
}


/************************************************************************/
/*						    Private Functions                           */
/************************************************************************/
void USARTClass::USART_Init()
{
	
}