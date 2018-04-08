#include "../include/usart.h"










// #if defined(ENABLE_UART1)
// 				{ USART1, uart1 },
// 					#endif
// 					#if defined(ENABLE_UART2)
// 				{ USART2, uart2 },
// 					#endif
// 					#if defined(ENABLE_UART3)
// 				{ USART3, uart3 },
// 					#endif



// #if defined(ENABLE_UART1) || defined(ENABLE_USART1)
// #ifdef USING_CHIMERA
// UARTClass_sPtr uart1;
// #else
// UARTClass_sPtr uart1 = boost::make_shared<UARTClass>(1);
// #endif
// 
// void USART1_IRQHandler(void)
// {
// 	if (uart1)
// 	{
// 		uart1->IRQHandler();
// 	}
// }
// #endif
// 
// #if defined(ENABLE_UART2) || defined(ENABLE_USART2)
// #ifdef USING_CHIMERA
// UARTClass_sPtr uart2;
// #else
// UARTClass_sPtr uart2 = boost::make_shared<UARTClass>(2);
// #endif
// 
// void USART2_IRQHandler(void)
// {
// 	if (uart2)
// 	{
// 		uart2->IRQHandler();
// 	}
// }
// #endif
// 
// 
// 
// #if defined(ENABLE_UART3) || defined(ENABLE_USART3)
// #ifdef USING_CHIMERA
// UARTClass_sPtr uart3;
// #else
// UARTClass_sPtr uart3 = boost::make_shared<UARTClass>(3);
// #endif
// 
// void USART3_IRQHandler(void)
// {
// 	if (uart3)
// 	{
// 		uart3->IRQHandler();
// 	}
// }
// #endif
// 
// #if defined(ENABLE_UART6) || defined(ENABLE_USART6)
// #ifdef USING_CHIMERA
// UARTClass_sPtr uart6;
// #else
// UARTClass_sPtr uart6 = boost::make_shared<UARTClass>(6);
// #endif
// 
// void USART6_IRQHandler(void)
// {
// 	if (uart6)
// 	{
// 		uart6->IRQHandler();
// 	}
// }
// #endif