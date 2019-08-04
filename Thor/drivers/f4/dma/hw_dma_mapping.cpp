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
  /* clang-format off */
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

  const Thor::Driver::RCC::ResourceMap_t StreamToRegisterIndex{
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ), 0 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ), 1 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ), 2 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ), 3 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ), 4 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ), 5 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ), 6 }, 
    { reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ), 7 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ), 0 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ), 1 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ), 2 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ), 3 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ), 4 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ), 5 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ), 6 }, 
    { reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ), 7 }, 
  };

  const std::unordered_map<Chimera::DMA::TransferDirection, uint32_t> TransferMap{
    { Chimera::DMA::TransferDirection::PERIPH_TO_MEMORY, Configuration::TransferDirection::P2M },
    { Chimera::DMA::TransferDirection::MEMORY_TO_PERIPH, Configuration::TransferDirection::M2P },
    { Chimera::DMA::TransferDirection::MEMORY_TO_MEMORY, Configuration::TransferDirection::M2M },
    { Chimera::DMA::TransferDirection::PERIPH_TO_PERIPH, Configuration::TransferDirection::INVALID }
  };

  const std::unordered_map<Chimera::DMA::Mode, uint32_t> ModeMap{
    { Chimera::DMA::Mode::NORMAL,           Configuration::Mode::Normal   },
    { Chimera::DMA::Mode::CIRCULAR,         Configuration::Mode::Circular },
    { Chimera::DMA::Mode::PERIPH_CONTROL,   Configuration::Mode::Periph   }
  };

  const std::unordered_map<Chimera::DMA::Channel, uint32_t> ChannelMap{
    { Chimera::DMA::Channel::CHANNEL0,    Configuration::ChannelSelect::Channel0 },
    { Chimera::DMA::Channel::CHANNEL1,    Configuration::ChannelSelect::Channel1 },
    { Chimera::DMA::Channel::CHANNEL2,    Configuration::ChannelSelect::Channel2 },
    { Chimera::DMA::Channel::CHANNEL3,    Configuration::ChannelSelect::Channel3 },
    { Chimera::DMA::Channel::CHANNEL4,    Configuration::ChannelSelect::Channel4 },
    { Chimera::DMA::Channel::CHANNEL5,    Configuration::ChannelSelect::Channel5 },
    { Chimera::DMA::Channel::CHANNEL6,    Configuration::ChannelSelect::Channel6 },
    { Chimera::DMA::Channel::CHANNEL7,    Configuration::ChannelSelect::Channel7 },
    { Chimera::DMA::Channel::CHANNEL8,    Configuration::ChannelSelect::Channel0 },
    { Chimera::DMA::Channel::CHANNEL9,    Configuration::ChannelSelect::Channel1 },
    { Chimera::DMA::Channel::CHANNEL10,   Configuration::ChannelSelect::Channel2 },
    { Chimera::DMA::Channel::CHANNEL11,   Configuration::ChannelSelect::Channel3 },
    { Chimera::DMA::Channel::CHANNEL12,   Configuration::ChannelSelect::Channel4 },
    { Chimera::DMA::Channel::CHANNEL13,   Configuration::ChannelSelect::Channel5 },
    { Chimera::DMA::Channel::CHANNEL14,   Configuration::ChannelSelect::Channel6 },
    { Chimera::DMA::Channel::CHANNEL15,   Configuration::ChannelSelect::Channel7 },
  };

  const std::unordered_map<Chimera::DMA::Channel, uint32_t> InstanceMap{
    { Chimera::DMA::Channel::CHANNEL0,  0 },
    { Chimera::DMA::Channel::CHANNEL1,  0 },
    { Chimera::DMA::Channel::CHANNEL2,  0 },
    { Chimera::DMA::Channel::CHANNEL3,  0 },
    { Chimera::DMA::Channel::CHANNEL4,  0 },
    { Chimera::DMA::Channel::CHANNEL5,  0 },
    { Chimera::DMA::Channel::CHANNEL6,  0 },
    { Chimera::DMA::Channel::CHANNEL7,  0 },
    { Chimera::DMA::Channel::CHANNEL8,  1 },
    { Chimera::DMA::Channel::CHANNEL9,  1 },
    { Chimera::DMA::Channel::CHANNEL10, 1 },
    { Chimera::DMA::Channel::CHANNEL11, 1 },
    { Chimera::DMA::Channel::CHANNEL12, 1 },
    { Chimera::DMA::Channel::CHANNEL13, 1 },
    { Chimera::DMA::Channel::CHANNEL14, 1 },
    { Chimera::DMA::Channel::CHANNEL15, 1 }
  };

  const std::unordered_map<Chimera::DMA::MemoryIncrement, uint32_t> MemoryIncrementMap{
    { Chimera::DMA::MemoryIncrement::ENABLED,   Configuration::MemoryIncrementMode::Increment   },
    { Chimera::DMA::MemoryIncrement::DISABLED,  Configuration::MemoryIncrementMode::Fixed       }
  };

  const std::unordered_map<Chimera::DMA::MemoryAlignment, uint32_t> MemoryAlignmentMap{
    { Chimera::DMA::MemoryAlignment::ALIGN_BYTE,      Configuration::MemoryDataSize::Byte       },
    { Chimera::DMA::MemoryAlignment::ALIGN_HALF_WORD, Configuration::MemoryDataSize::HalfWord   },
    { Chimera::DMA::MemoryAlignment::ALIGN_WORD,      Configuration::MemoryDataSize::Word       }
  };

  const std::unordered_map<Chimera::DMA::PeripheralIncrement, uint32_t> PeripheralIncrementMap{
    { Chimera::DMA::PeripheralIncrement::ENABLED,   Configuration::PeriphIncrementMode::Increment   },
    { Chimera::DMA::PeripheralIncrement::DISABLED,  Configuration::PeriphIncrementMode::Fixed       }
  };

  const std::unordered_map<Chimera::DMA::PeripheralAlignment, uint32_t> PeripheralAlignmentMap{
    { Chimera::DMA::PeripheralAlignment::ALIGN_BYTE,      Configuration::PeriphDataSize::Byte       },
    { Chimera::DMA::PeripheralAlignment::ALIGN_HALF_WORD, Configuration::PeriphDataSize::HalfWord   },
    { Chimera::DMA::PeripheralAlignment::ALIGN_WORD,      Configuration::PeriphDataSize::Word       }
  };

  const std::unordered_map<Chimera::DMA::Priority, uint32_t> PriorityMap{
    { Chimera::DMA::Priority::LOW,        Configuration::PriorityLevel::Low     },
    { Chimera::DMA::Priority::MEDIUM,     Configuration::PriorityLevel::Medium  },
    { Chimera::DMA::Priority::HIGH,       Configuration::PriorityLevel::High    },
    { Chimera::DMA::Priority::VERY_HIGH,  Configuration::PriorityLevel::Ultra   }
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

  const std::unordered_map<Chimera::DMA::Channel, StreamX *const> ChannelToStream{
    { Chimera::DMA::Channel::CHANNEL0, DMA1_STREAM0 },
    { Chimera::DMA::Channel::CHANNEL1, DMA1_STREAM1 },
    { Chimera::DMA::Channel::CHANNEL2, DMA1_STREAM2 },
    { Chimera::DMA::Channel::CHANNEL3, DMA1_STREAM3 },
    { Chimera::DMA::Channel::CHANNEL4, DMA1_STREAM4 },
    { Chimera::DMA::Channel::CHANNEL5, DMA1_STREAM5 },
    { Chimera::DMA::Channel::CHANNEL6, DMA1_STREAM6 },
    { Chimera::DMA::Channel::CHANNEL7, DMA1_STREAM7 },
    { Chimera::DMA::Channel::CHANNEL8, DMA2_STREAM0 },
    { Chimera::DMA::Channel::CHANNEL9, DMA2_STREAM1 },
    { Chimera::DMA::Channel::CHANNEL10, DMA2_STREAM2 },
    { Chimera::DMA::Channel::CHANNEL11, DMA2_STREAM3 },
    { Chimera::DMA::Channel::CHANNEL12, DMA2_STREAM4 },
    { Chimera::DMA::Channel::CHANNEL13, DMA2_STREAM5 },
    { Chimera::DMA::Channel::CHANNEL14, DMA2_STREAM6 },
    { Chimera::DMA::Channel::CHANNEL15, DMA2_STREAM7 }
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
}
#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
