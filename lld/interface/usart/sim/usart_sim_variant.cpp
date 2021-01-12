/********************************************************************************
 *  File Name:
 *    usart_sim_variant.cpp
 *
 *  Description:
 *    Simulator variant of the USART driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/interface/usart/usart_prv_data.hpp>
#include <Thor/lld/interface/usart/usart_intf.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{


  {
  }
}    // namespace Thor::LLD::USART

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_usart_mapping.hpp
  ------------------------------------------------*/
  // RegisterConfig USART_ClockConfig[ Thor::LLD::USART::NUM_USART_PERIPHS ];
  // RegisterConfig USART_ClockConfigLP[ Thor::LLD::USART::NUM_USART_PERIPHS ];
  // RegisterConfig USART_ResetConfig[ Thor::LLD::USART::NUM_USART_PERIPHS ];
  // Chimera::Clock::Bus USART_SourceClock[ Thor::LLD::USART::NUM_USART_PERIPHS ];

  // PCC USARTLookup = { USART_ClockConfig,
  //                     nullptr,
  //                     USART_ResetConfig,
  //                     USART_SourceClock,
  //                     Thor::LLD::USART::NUM_USART_PERIPHS,
  //                     Thor::LLD::USART::getResourceIndex
  //                     };

  void USARTInit()
  {
    using namespace Thor::LLD::USART;

    // /*------------------------------------------------
    // USART clock enable register access lookup table
    // ------------------------------------------------*/
    // #if defined ( STM32_USART1_PERIPH_AVAILABLE )
    // USART_ClockConfig[ USART1_RESOURCE_INDEX ].mask = APB2ENR_USART1EN;
    // USART_ClockConfig[ USART1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;
    // #endif

    // #if defined ( STM32_USART2_PERIPH_AVAILABLE )
    // USART_ClockConfig[ USART2_RESOURCE_INDEX ].mask = APB1ENR1_USART2EN;
    // USART_ClockConfig[ USART2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
    // #endif

    // #if defined ( STM32_USART3_PERIPH_AVAILABLE )
    // USART_ClockConfig[ USART3_RESOURCE_INDEX ].mask = APB1ENR1_USART3EN;
    // USART_ClockConfig[ USART3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
    // #endif

    // /*------------------------------------------------
    // USART reset register access lookup table
    // ------------------------------------------------*/
    // #if defined ( STM32_USART1_PERIPH_AVAILABLE )
    // USART_ResetConfig[ USART1_RESOURCE_INDEX ].mask = APB2RSTR_USART1RST;
    // USART_ResetConfig[ USART1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;
    // #endif

    // #if defined ( STM32_USART2_PERIPH_AVAILABLE )
    // USART_ResetConfig[ USART2_RESOURCE_INDEX ].mask = APB1RSTR1_USART2RST;
    // USART_ResetConfig[ USART2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;
    // #endif

    // #if defined ( STM32_USART3_PERIPH_AVAILABLE )
    // USART_ResetConfig[ USART3_RESOURCE_INDEX ].mask = APB1RSTR1_USART3RST;
    // USART_ResetConfig[ USART3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;
    // #endif

    // /*------------------------------------------------
    // USART clocking bus source identifier
    // ------------------------------------------------*/
    // #if defined ( STM32_USART1_PERIPH_AVAILABLE )
    // USART_SourceClock[ USART1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB2;
    // #endif

    // #if defined ( STM32_USART2_PERIPH_AVAILABLE )
    // USART_SourceClock[ USART2_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
    // #endif

    // #if defined ( STM32_USART3_PERIPH_AVAILABLE )
    // USART_SourceClock[ USART3_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
    // #endif
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_USART */
