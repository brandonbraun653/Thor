/********************************************************************************
 *  File Name:
 *    hw_uart_data.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/usart>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{
}

#endif /* TARGET_STM32F4 && THOR_LLD_USART */
