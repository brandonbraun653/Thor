/********************************************************************************
 *  File Name:
 *    hw_dma_register_stm32f446xx.cpp
 *
 *  Description:
 *
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C/C++ Includes */


/* Thor Includes */

#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>
#include <Thor/drivers/f4/dma/hw_dma_register_stm32f446xx.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
#if defined( EMBEDDED )
  RegisterMap *DMA1_PERIPH = reinterpret_cast<RegisterMap *>( DMA1_BASE_ADDR );
  RegisterMap *DMA2_PERIPH = reinterpret_cast<RegisterMap *>( DMA2_BASE_ADDR );

  StreamX *DMA1_STREAM0 = reinterpret_cast<StreamX *>( DMA1_STREAM0_BASE_ADDR );
  StreamX *DMA1_STREAM1 = reinterpret_cast<StreamX *>( DMA1_STREAM1_BASE_ADDR );
  StreamX *DMA1_STREAM2 = reinterpret_cast<StreamX *>( DMA1_STREAM2_BASE_ADDR );
  StreamX *DMA1_STREAM3 = reinterpret_cast<StreamX *>( DMA1_STREAM3_BASE_ADDR );
  StreamX *DMA1_STREAM4 = reinterpret_cast<StreamX *>( DMA1_STREAM4_BASE_ADDR );
  StreamX *DMA1_STREAM5 = reinterpret_cast<StreamX *>( DMA1_STREAM5_BASE_ADDR );
  StreamX *DMA1_STREAM6 = reinterpret_cast<StreamX *>( DMA1_STREAM6_BASE_ADDR );
  StreamX *DMA1_STREAM7 = reinterpret_cast<StreamX *>( DMA1_STREAM7_BASE_ADDR );

  StreamX *DMA2_STREAM0 = reinterpret_cast<StreamX *>( DMA2_STREAM0_BASE_ADDR );
  StreamX *DMA2_STREAM1 = reinterpret_cast<StreamX *>( DMA2_STREAM1_BASE_ADDR );
  StreamX *DMA2_STREAM2 = reinterpret_cast<StreamX *>( DMA2_STREAM2_BASE_ADDR );
  StreamX *DMA2_STREAM3 = reinterpret_cast<StreamX *>( DMA2_STREAM3_BASE_ADDR );
  StreamX *DMA2_STREAM4 = reinterpret_cast<StreamX *>( DMA2_STREAM4_BASE_ADDR );
  StreamX *DMA2_STREAM5 = reinterpret_cast<StreamX *>( DMA2_STREAM5_BASE_ADDR );
  StreamX *DMA2_STREAM6 = reinterpret_cast<StreamX *>( DMA2_STREAM6_BASE_ADDR );
  StreamX *DMA2_STREAM7 = reinterpret_cast<StreamX *>( DMA2_STREAM7_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( DMA1_PERIPH ), DMA1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( DMA2_PERIPH ), DMA2_RESOURCE_INDEX }
  };

#elif defined( _SIM )
  RegisterMap *DMA1_PERIPH = nullptr;
  RegisterMap *DMA2_PERIPH = nullptr;

  StreamX *DMA1_STREAM0 = nullptr;
  StreamX *DMA1_STREAM1 = nullptr;
  StreamX *DMA1_STREAM2 = nullptr;
  StreamX *DMA1_STREAM3 = nullptr;
  StreamX *DMA1_STREAM4 = nullptr;
  StreamX *DMA1_STREAM5 = nullptr;
  StreamX *DMA1_STREAM6 = nullptr;
  StreamX *DMA1_STREAM7 = nullptr;

  StreamX *DMA2_STREAM0 = nullptr;
  StreamX *DMA2_STREAM1 = nullptr;
  StreamX *DMA2_STREAM2 = nullptr;
  StreamX *DMA2_STREAM3 = nullptr;
  StreamX *DMA2_STREAM4 = nullptr;
  StreamX *DMA2_STREAM5 = nullptr;
  StreamX *DMA2_STREAM6 = nullptr;
  StreamX *DMA2_STREAM7 = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
#endif

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> StreamToResourceIndex;
  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> StreamToRegisterIndex;

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate memory for the peripheral registers
    ------------------------------------------------*/
    DMA1_PERIPH = new RegisterMap;
    DMA2_PERIPH = new RegisterMap;

    DMA1_STREAM0 = &( DMA1_PERIPH->STREAM0 );
    DMA1_STREAM1 = &( DMA1_PERIPH->STREAM1 );
    DMA1_STREAM2 = &( DMA1_PERIPH->STREAM2 );
    DMA1_STREAM3 = &( DMA1_PERIPH->STREAM3 );
    DMA1_STREAM4 = &( DMA1_PERIPH->STREAM4 );
    DMA1_STREAM5 = &( DMA1_PERIPH->STREAM5 );
    DMA1_STREAM6 = &( DMA1_PERIPH->STREAM6 );
    DMA1_STREAM7 = &( DMA1_PERIPH->STREAM7 );

    DMA2_STREAM0 = &( DMA2_PERIPH->STREAM0 );
    DMA2_STREAM1 = &( DMA2_PERIPH->STREAM1 );
    DMA2_STREAM2 = &( DMA2_PERIPH->STREAM2 );
    DMA2_STREAM3 = &( DMA2_PERIPH->STREAM3 );
    DMA2_STREAM4 = &( DMA2_PERIPH->STREAM4 );
    DMA2_STREAM5 = &( DMA2_PERIPH->STREAM5 );
    DMA2_STREAM6 = &( DMA2_PERIPH->STREAM6 );
    DMA2_STREAM7 = &( DMA2_PERIPH->STREAM7 );

    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_PERIPH ), DMA1_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_PERIPH ), DMA2_RESOURCE_INDEX );
#endif

    /*------------------------------------------------
    Initialize peripheral instances
    ------------------------------------------------*/
    periphInstanceList[ DMA1_RESOURCE_INDEX ] = DMA1_PERIPH;
    periphInstanceList[ DMA2_RESOURCE_INDEX ] = DMA2_PERIPH;

    /*------------------------------------------------
    Initialize stream instances
    ------------------------------------------------*/
    streamInstanceList[ DMA1_STREAM0_RESOURCE_INDEX ] = DMA1_STREAM0;
    streamInstanceList[ DMA1_STREAM1_RESOURCE_INDEX ] = DMA1_STREAM1;
    streamInstanceList[ DMA1_STREAM2_RESOURCE_INDEX ] = DMA1_STREAM2;
    streamInstanceList[ DMA1_STREAM3_RESOURCE_INDEX ] = DMA1_STREAM3;
    streamInstanceList[ DMA1_STREAM4_RESOURCE_INDEX ] = DMA1_STREAM4;
    streamInstanceList[ DMA1_STREAM5_RESOURCE_INDEX ] = DMA1_STREAM5;
    streamInstanceList[ DMA1_STREAM6_RESOURCE_INDEX ] = DMA1_STREAM6;
    streamInstanceList[ DMA1_STREAM7_RESOURCE_INDEX ] = DMA1_STREAM7;

    streamInstanceList[ DMA2_STREAM0_RESOURCE_INDEX ] = DMA2_STREAM0;
    streamInstanceList[ DMA2_STREAM1_RESOURCE_INDEX ] = DMA2_STREAM1;
    streamInstanceList[ DMA2_STREAM2_RESOURCE_INDEX ] = DMA2_STREAM2;
    streamInstanceList[ DMA2_STREAM3_RESOURCE_INDEX ] = DMA2_STREAM3;
    streamInstanceList[ DMA2_STREAM4_RESOURCE_INDEX ] = DMA2_STREAM4;
    streamInstanceList[ DMA2_STREAM5_RESOURCE_INDEX ] = DMA2_STREAM5;
    streamInstanceList[ DMA2_STREAM6_RESOURCE_INDEX ] = DMA2_STREAM6;
    streamInstanceList[ DMA2_STREAM7_RESOURCE_INDEX ] = DMA2_STREAM7;

    /*------------------------------------------------
    Initialize the Stream->Resource Index Mapping
    ------------------------------------------------*/
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ), DMA1_STREAM0_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ), DMA1_STREAM1_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ), DMA1_STREAM2_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ), DMA1_STREAM3_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ), DMA1_STREAM4_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ), DMA1_STREAM5_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM6 ), DMA1_STREAM6_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ), DMA1_STREAM7_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ), DMA2_STREAM0_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ), DMA2_STREAM1_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ), DMA2_STREAM2_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ), DMA2_STREAM3_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ), DMA2_STREAM4_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ), DMA2_STREAM5_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM6 ), DMA2_STREAM6_RESOURCE_INDEX );
    StreamToResourceIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ), DMA2_STREAM7_RESOURCE_INDEX );

    /*------------------------------------------------
    Initialize the Stream->Register Index Mapping
    ------------------------------------------------*/
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ), 0 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ), 1 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ), 2 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ), 3 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ), 4 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ), 5 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM6 ), 6 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ), 7 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ), 0 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ), 1 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ), 2 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ), 3 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ), 4 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ), 5 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM6 ), 6 );
    StreamToRegisterIndex.append( reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ), 7 );
  }

}    // namespace Thor::Driver::DMA

namespace Thor::Driver::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_dma_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig DMA_ClockConfig[ dmaTableSize ];
  RegisterConfig DMA_ClockConfigLP[ dmaTableSize ];
  RegisterConfig DMA_ResetConfig[ dmaTableSize ];
  Configuration::ClockType_t DMA_SourceClock[ dmaTableSize ];

  const PCC DMALookup = {
    DMA_ClockConfig, DMA_ClockConfigLP, DMA_ResetConfig, DMA_SourceClock, &Thor::Driver::DMA::InstanceToResourceIndex,
    dmaTableSize
  };

  void DMAInit()
  {
    using namespace Thor::Driver::DMA;

    /*------------------------------------------------
    DMA clock enable register access lookup table
    ------------------------------------------------*/
    DMA_ClockConfig[ DMA1_RESOURCE_INDEX ].mask = AHB1ENR_DMA1EN;
    DMA_ClockConfig[ DMA1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    DMA_ClockConfig[ DMA2_RESOURCE_INDEX ].mask = AHB1ENR_DMA2EN;
    DMA_ClockConfig[ DMA2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    /*------------------------------------------------
    DMA low power clock enable register access lookup table
    ------------------------------------------------*/
    DMA_ClockConfigLP[ DMA1_RESOURCE_INDEX ].mask = AHB1LPENR_DMA1LPEN;
    DMA_ClockConfigLP[ DMA1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    DMA_ClockConfigLP[ DMA2_RESOURCE_INDEX ].mask = AHB1LPENR_DMA2LPEN;
    DMA_ClockConfigLP[ DMA2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    /*------------------------------------------------
    DMA reset register access lookup table
    ------------------------------------------------*/
    DMA_ResetConfig[ DMA1_RESOURCE_INDEX ].mask = AHB1RSTR_DMA1RST;
    DMA_ResetConfig[ DMA1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    DMA_ResetConfig[ DMA2_RESOURCE_INDEX ].mask = AHB1RSTR_DMA2RST;
    DMA_ResetConfig[ DMA2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    /*------------------------------------------------
    DMA clocking bus source identifier
    ------------------------------------------------*/
    DMA_SourceClock[ DMA1_RESOURCE_INDEX ] = Configuration::ClockType::HCLK;
    DMA_SourceClock[ DMA2_RESOURCE_INDEX ] = Configuration::ClockType::HCLK;
  };

}    // namespace Thor::Driver::RCC::LookupTables

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */