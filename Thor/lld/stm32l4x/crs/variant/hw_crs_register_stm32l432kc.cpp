/********************************************************************************
 *  File Name:
 *    hw_crs_register_stm32l432kc.cpp
 *
 *  Description:
 *    CRS register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/crs/hw_crs_prj.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_prv_data.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32l4x/crs/variant/hw_crs_register_stm32l4xxxx.hpp>


namespace Thor::LLD::CRS
{
  static RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_CRS1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( CRS1_PERIPH ) )
    {
      return CRS1_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }

}    // namespace Thor::LLD::CRS


namespace Thor::LLD::RCC::LookupTables
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  RegisterConfig CRS_ClockConfig[ Thor::LLD::CRS::NUM_CRS_PERIPHS ];
  RegisterConfig CRS_ResetConfig[ Thor::LLD::CRS::NUM_CRS_PERIPHS ];
  Chimera::Clock::Bus CRS_SourceClock[ Thor::LLD::CRS::NUM_CRS_PERIPHS ];

  PCC CRSLookup = { CRS_ClockConfig,
                    nullptr,
                    CRS_ResetConfig,
                    CRS_SourceClock,
                    Thor::LLD::CRS::NUM_CRS_PERIPHS,
                    Thor::LLD::CRS::getResourceIndex };

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void CRSInit()
  {
    using namespace Thor::LLD::CRS;

    /*------------------------------------------------
    CRS clock enable register access lookup table
    ------------------------------------------------*/
#if defined( STM32_CRS1_PERIPH_AVAILABLE )
    CRS_ClockConfig[ CRS1_RESOURCE_INDEX ].mask = APB1ENR1_CRSEN;
    CRS_ClockConfig[ CRS1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
#endif

    /*------------------------------------------------
    CRS reset register access lookup table
    ------------------------------------------------*/
#if defined( STM32_CRS1_PERIPH_AVAILABLE )
    CRS_ResetConfig[ CRS1_RESOURCE_INDEX ].mask = APB1RSTR1_CRSRST;
    CRS_ResetConfig[ CRS1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;
#endif

    /*------------------------------------------------
    CRS clocking bus source identifier
    ------------------------------------------------*/
#if defined( STM32_CRS1_PERIPH_AVAILABLE )
    CRS_SourceClock[ CRS1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
#endif
  };
}    // namespace Thor::LLD::RCC::LookupTables