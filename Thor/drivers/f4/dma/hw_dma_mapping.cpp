/********************************************************************************
 *   File Name:
 *    hw_dma_mapping.hpp
 *
 *   Description:
 *    STM32 Mappings for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
  const Thor::Driver::RCC::ResourceMap_t InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( DMA1_PERIPH ), 0 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_PERIPH ), 1 }
  };

  const Thor::Driver::RCC::ResourceMap_t StreamToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ), 0 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ), 1 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ), 2 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ), 3 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ), 4 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ), 5 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ), 6 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ), 7 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ), 8 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ), 9 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ), 10 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ), 11 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ), 12 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ), 13 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ), 14 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ), 15 }, 
  };

  const IRQn_Type DMAStream_IRQn[ NUM_DMA_STREAMS * NUM_DMA_PERIPHS ] = {
    DMA1_Stream0_IRQn,
    DMA1_Stream1_IRQn,
    DMA1_Stream2_IRQn,
    DMA1_Stream3_IRQn,
    DMA1_Stream4_IRQn,
    DMA1_Stream5_IRQn,
    DMA1_Stream6_IRQn,
    DMA1_Stream7_IRQn,
    DMA2_Stream0_IRQn,
    DMA2_Stream1_IRQn,
    DMA2_Stream2_IRQn,
    DMA2_Stream3_IRQn,
    DMA2_Stream4_IRQn,
    DMA2_Stream5_IRQn,
    DMA2_Stream6_IRQn,
    DMA2_Stream7_IRQn
  };

}
#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
