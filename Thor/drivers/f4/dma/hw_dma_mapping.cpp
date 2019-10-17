/********************************************************************************
 *   File Name:
 *    hw_dma_mapping.hpp
 *
 *   Description:
 *    STM32 Mappings for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/types/dma_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
#if defined( _EMBEDDED )
  RegisterMap *const DMA1_PERIPH = reinterpret_cast<RegisterMap *const>( DMA1_BASE_ADDR );
  RegisterMap *const DMA2_PERIPH = reinterpret_cast<RegisterMap *const>( DMA2_BASE_ADDR );

  StreamX *const DMA1_STREAM0 = reinterpret_cast<StreamX *const>( DMA1_STREAM0_BASE_ADDR );
  StreamX *const DMA1_STREAM1 = reinterpret_cast<StreamX *const>( DMA1_STREAM1_BASE_ADDR );
  StreamX *const DMA1_STREAM2 = reinterpret_cast<StreamX *const>( DMA1_STREAM2_BASE_ADDR );
  StreamX *const DMA1_STREAM3 = reinterpret_cast<StreamX *const>( DMA1_STREAM3_BASE_ADDR );
  StreamX *const DMA1_STREAM4 = reinterpret_cast<StreamX *const>( DMA1_STREAM4_BASE_ADDR );
  StreamX *const DMA1_STREAM5 = reinterpret_cast<StreamX *const>( DMA1_STREAM5_BASE_ADDR );
  StreamX *const DMA1_STREAM6 = reinterpret_cast<StreamX *const>( DMA1_STREAM6_BASE_ADDR );
  StreamX *const DMA1_STREAM7 = reinterpret_cast<StreamX *const>( DMA1_STREAM7_BASE_ADDR );

  StreamX *const DMA2_STREAM0 = reinterpret_cast<StreamX *const>( DMA2_STREAM0_BASE_ADDR );
  StreamX *const DMA2_STREAM1 = reinterpret_cast<StreamX *const>( DMA2_STREAM1_BASE_ADDR );
  StreamX *const DMA2_STREAM2 = reinterpret_cast<StreamX *const>( DMA2_STREAM2_BASE_ADDR );
  StreamX *const DMA2_STREAM3 = reinterpret_cast<StreamX *const>( DMA2_STREAM3_BASE_ADDR );
  StreamX *const DMA2_STREAM4 = reinterpret_cast<StreamX *const>( DMA2_STREAM4_BASE_ADDR );
  StreamX *const DMA2_STREAM5 = reinterpret_cast<StreamX *const>( DMA2_STREAM5_BASE_ADDR );
  StreamX *const DMA2_STREAM6 = reinterpret_cast<StreamX *const>( DMA2_STREAM6_BASE_ADDR );
  StreamX *const DMA2_STREAM7 = reinterpret_cast<StreamX *const>( DMA2_STREAM7_BASE_ADDR );

#elif defined( _SIM )
  RegisterMap *const DMA1_PERIPH = new RegisterMap;
  RegisterMap *const DMA2_PERIPH = new RegisterMap;

  StreamX *const DMA1_STREAM0 = new StreamX;
  StreamX *const DMA1_STREAM1 = new StreamX;
  StreamX *const DMA1_STREAM2 = new StreamX;
  StreamX *const DMA1_STREAM3 = new StreamX;
  StreamX *const DMA1_STREAM4 = new StreamX;
  StreamX *const DMA1_STREAM5 = new StreamX;
  StreamX *const DMA1_STREAM6 = new StreamX;
  StreamX *const DMA1_STREAM7 = new StreamX;

  StreamX *const DMA2_STREAM0 = new StreamX;
  StreamX *const DMA2_STREAM1 = new StreamX;
  StreamX *const DMA2_STREAM2 = new StreamX;
  StreamX *const DMA2_STREAM3 = new StreamX;
  StreamX *const DMA2_STREAM4 = new StreamX;
  StreamX *const DMA2_STREAM5 = new StreamX;
  StreamX *const DMA2_STREAM6 = new StreamX;
  StreamX *const DMA2_STREAM7 = new StreamX;

#endif 

  const std::array<RegisterMap *const, NUM_DMA_PERIPHS> periphInstanceList = { DMA1_PERIPH, DMA2_PERIPH };

  const std::array<StreamX *const, NUM_DMA_STREAMS> streamInstanceList = {
    DMA1_STREAM0, DMA1_STREAM1, DMA1_STREAM2, DMA1_STREAM3, DMA1_STREAM4, DMA1_STREAM5, DMA1_STREAM6, DMA1_STREAM7,
    DMA2_STREAM0, DMA2_STREAM1, DMA2_STREAM2, DMA2_STREAM3, DMA2_STREAM4, DMA2_STREAM5, DMA2_STREAM6, DMA2_STREAM7
  };

  /* clang-format off */
  const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( DMA1_PERIPH ), 0 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_PERIPH ), 1 }
  };

  const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> StreamToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ), DMA1_STREAM0_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ), DMA1_STREAM1_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ), DMA1_STREAM2_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ), DMA1_STREAM3_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ), DMA1_STREAM4_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ), DMA1_STREAM5_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM6 ), DMA1_STREAM6_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ), DMA1_STREAM7_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ), DMA2_STREAM0_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ), DMA2_STREAM1_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ), DMA2_STREAM2_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ), DMA2_STREAM3_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ), DMA2_STREAM4_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ), DMA2_STREAM5_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM6 ), DMA2_STREAM6_RESOURCE_INDEX }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ), DMA2_STREAM7_RESOURCE_INDEX }, 
  };

  const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> StreamToRegisterIndex{
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ), 0 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ), 1 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ), 2 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ), 3 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ), 4 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ), 5 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM6 ), 6 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ), 7 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ), 0 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ), 1 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ), 2 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ), 3 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ), 4 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ), 5 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM6 ), 6 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ), 7 }, 
  };

  const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::TransferDirection::NUM_OPTIONS)> TransferMap{
    Configuration::TransferDirection::P2M,   
    Configuration::TransferDirection::M2P,   
    Configuration::TransferDirection::M2M,   
    Configuration::TransferDirection::INVALID
  };

  const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::Mode::NUM_OPTIONS)> ModeMap{
    Configuration::Mode::Normal  ,
    Configuration::Mode::Circular,
    Configuration::Mode::Periph   
  };

  const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::MemoryIncrement::NUM_OPTIONS)> MemoryIncrementMap{
    Configuration::MemoryIncrementMode::Increment,
    Configuration::MemoryIncrementMode::Fixed    
  };

  const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::MemoryAlignment::NUM_OPTIONS)> MemoryAlignmentMap{
    Configuration::MemoryDataSize::Byte    ,
    Configuration::MemoryDataSize::HalfWord,
    Configuration::MemoryDataSize::Word    
  };

  const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::PeripheralIncrement::NUM_OPTIONS)> PeripheralIncrementMap{
    Configuration::PeriphIncrementMode::Increment,
    Configuration::PeriphIncrementMode::Fixed    
  };

  const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::PeripheralAlignment::NUM_OPTIONS)> PeripheralAlignmentMap{
    Configuration::PeriphDataSize::Byte    ,
    Configuration::PeriphDataSize::HalfWord,
    Configuration::PeriphDataSize::Word    
  };

  const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::Priority::NUM_OPTIONS)> PriorityMap{
    Configuration::PriorityLevel::Low   ,
    Configuration::PriorityLevel::Medium,
    Configuration::PriorityLevel::High  ,
    Configuration::PriorityLevel::Ultra 
  };

  const IRQn_Type DMAStream_IRQn[ NUM_DMA_STREAMS ] = {
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

  const uint32_t DMAStream_TCIF[ NUM_DMA_STREAMS ] = {
    LISR_TCIF0,
    LISR_TCIF1,
    LISR_TCIF2,
    LISR_TCIF3,
    HISR_TCIF4,
    HISR_TCIF5,
    HISR_TCIF6,
    HISR_TCIF7,
    LISR_TCIF0,
    LISR_TCIF1,
    LISR_TCIF2,
    LISR_TCIF3,
    HISR_TCIF4,
    HISR_TCIF5,
    HISR_TCIF6,
    HISR_TCIF7
  };

  const uint32_t DMAStream_HTIF[ NUM_DMA_STREAMS ] = {
    LISR_HTIF0,
    LISR_HTIF1,
    LISR_HTIF2,
    LISR_HTIF3,
    HISR_HTIF4,
    HISR_HTIF5,
    HISR_HTIF6,
    HISR_HTIF7,
    LISR_HTIF0,
    LISR_HTIF1,
    LISR_HTIF2,
    LISR_HTIF3,
    HISR_HTIF4,
    HISR_HTIF5,
    HISR_HTIF6,
    HISR_HTIF7
  };

  const uint32_t DMAStream_TEIF[ NUM_DMA_STREAMS ] = {
    LISR_TEIF0,
    LISR_TEIF1,
    LISR_TEIF2,
    LISR_TEIF3,
    HISR_TEIF4,
    HISR_TEIF5,
    HISR_TEIF6,
    HISR_TEIF7,
    LISR_TEIF0,
    LISR_TEIF1,
    LISR_TEIF2,
    LISR_TEIF3,
    HISR_TEIF4,
    HISR_TEIF5,
    HISR_TEIF6,
    HISR_TEIF7
  };

  const uint32_t DMAStream_DMEIF[ NUM_DMA_STREAMS ] = {
    LISR_DMEIF0,
    LISR_DMEIF1,
    LISR_DMEIF2,
    LISR_DMEIF3,
    HISR_DMEIF4,
    HISR_DMEIF5,
    HISR_DMEIF6,
    HISR_DMEIF7,
    LISR_DMEIF0,
    LISR_DMEIF1,
    LISR_DMEIF2,
    LISR_DMEIF3,
    HISR_DMEIF4,
    HISR_DMEIF5,
    HISR_DMEIF6,
    HISR_DMEIF7
  };

  const uint32_t DMAStream_FEIF[ NUM_DMA_STREAMS ] = {
    LISR_FEIF0,
    LISR_FEIF1,
    LISR_FEIF2,
    LISR_FEIF3,
    HISR_FEIF4,
    HISR_FEIF5,
    HISR_FEIF6,
    HISR_FEIF7,
    LISR_FEIF0,
    LISR_FEIF1,
    LISR_FEIF2,
    LISR_FEIF3,
    HISR_FEIF4,
    HISR_FEIF5,
    HISR_FEIF6,
    HISR_FEIF7
  };

  /* clang-format on */
}    // namespace Thor::Driver::DMA
#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
