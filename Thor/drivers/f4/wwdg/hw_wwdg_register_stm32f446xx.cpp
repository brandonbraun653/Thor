/********************************************************************************
  *  File Name:
  *    hw_wwdg_register_stm32f446xx.cpp
  *
  *  Description:
  *    Explicit STM32F446xx WWDG data and routines
  *
  *  2019 | Brandon Braun | brandonbraun653@gmail.com
  ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_driver.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_mapping.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_register_stm32f446xx.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 ) && defined( STM32F446xx )

namespace Thor::Driver::WWDG
{
#if defined( _EMBEDDED )
  RegisterMap * WWDG_PERIPH = reinterpret_cast<RegisterMap *>( WWDG1_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex = {
    { reinterpret_cast<std::uintptr_t>( WWDG_PERIPH ), WWDG1_RESOURCE_INDEX } 
  };

#elif defined( _SIM )
  RegisterMap *WWDG1_PERIPH = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
#endif

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    WWDG1_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ WWDG1_RESOURCE_INDEX ] = WWDG1_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( WWDG1_PERIPH ), WWDG1_RESOURCE_INDEX );

#endif
  }
}    // namespace Thor::Driver::WWDG

namespace Thor::Driver::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_wwdg_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig WWDG_ClockConfig[ wwdgTableSize ];
  RegisterConfig WWDG_ClockConfigLP[ wwdgTableSize ];
  RegisterConfig WWDG_ResetConfig[ wwdgTableSize ];
  Configuration::ClockType_t WWDG_SourceClock[ wwdgTableSize ];

  const PCC WWDGLookup = {
    WWDG_ClockConfig, WWDG_ClockConfigLP, WWDG_ResetConfig, WWDG_SourceClock, &Thor::Driver::WWDG::InstanceToResourceIndex,
    wwdgTableSize
  };

  void WWDGInit()
  {
    using namespace Thor::Driver::WWDG;

    /*------------------------------------------------
    WWDG clock enable register access lookup table
    ------------------------------------------------*/
    WWDG_ClockConfig[ WWDG1_RESOURCE_INDEX ].mask = APB1ENR_WWDGEN;
    WWDG_ClockConfig[ WWDG1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR;

    /*------------------------------------------------
    WWDG low power clock enable register access lookup table
    ------------------------------------------------*/
    WWDG_ClockConfigLP[ WWDG1_RESOURCE_INDEX ].mask = APB1LPENR_WWDGLPEN;
    WWDG_ClockConfigLP[ WWDG1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1LPENR;

    /*------------------------------------------------
    WWDG reset register access lookup table
    ------------------------------------------------*/
    WWDG_ResetConfig[ WWDG1_RESOURCE_INDEX ].mask = APB1RSTR_WWDGRST;
    WWDG_ResetConfig[ WWDG1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR;

    /*------------------------------------------------
    WWDG clocking bus source identifier
    ------------------------------------------------*/
    WWDG_SourceClock[ WWDG1_RESOURCE_INDEX ] = Configuration::ClockType::HCLK;
  };

}    // namespace Thor::Driver::RCC::LookupTables

#endif /* TARGET_STM32F4 && THOR_DRIVER_WWDG */