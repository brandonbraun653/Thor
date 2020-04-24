/********************************************************************************
 *  File Name:
 *    hw_flash_register_stm32l432kc.cpp
 *
 *  Description:
 *    FLASH register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/flash/hw_flash_driver.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_mapping.hpp>
#include <Thor/lld/stm32l4x/flash/variant/hw_flash_register_stm32l432kc.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_FLASH )

namespace Thor::LLD::FLASH
{
#if defined( EMBEDDED )
  /*-------------------------------------------------
  Memory Mapped Structs to Peripherals
  -------------------------------------------------*/
  RegisterMap *FLASH_PERIPH = reinterpret_cast<RegisterMap *>( FLASH_BASE_ADDR );

  /*-------------------------------------------------
  Lookup Tables Defintions
  -------------------------------------------------*/
   Thor::LLD::RIndexMap InstanceToResourceIndex{
     { reinterpret_cast<std::uintptr_t>( FLASH_PERIPH ), FLASH_RESOURCE_INDEX },
   };

#elif defined( _SIM )
  /*-------------------------------------------------
  Memory Mapped Structs to Virtual Peripherals
  -------------------------------------------------*/
  RegisterMap *FLASH_PERIPH = nullptr;

  /*-------------------------------------------------
  Lookup Tables Definitions
  -------------------------------------------------*/
  Thor::LLD::RIndexMap InstanceToResourceIndex;
#endif

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    FLASH_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ FLASH_RESOURCE_INDEX ] = FLASH_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( FLASH_PERIPH ), FLASH_RESOURCE_INDEX );

#endif
  }
}    // namespace Thor::LLD::GPIO

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_flash_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig FLASH_ClockConfig[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
  RegisterConfig FLASH_ResetConfig[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
  Chimera::Clock::Bus FLASH_SourceClock[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];

  PCC FLASHLookup = {
    FLASH_ClockConfig, nullptr, FLASH_ResetConfig, FLASH_SourceClock, &Thor::LLD::FLASH::InstanceToResourceIndex,
    Thor::LLD::FLASH::NUM_FLASH_PERIPHS
  };

  void FLASHInit()
  {
    using namespace Thor::LLD::FLASH;

    /*------------------------------------------------
    FLASH clock enable register access lookup table
    ------------------------------------------------*/
     FLASH_ClockConfig[ FLASH_RESOURCE_INDEX ].mask = AHB1ENR_FLASHEN;
     FLASH_ClockConfig[ FLASH_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    /*------------------------------------------------
    FLASH reset register access lookup table
    ------------------------------------------------*/
     FLASH_ResetConfig[ FLASH_RESOURCE_INDEX ].mask = AHB1RSTR_FLASHRST;
     FLASH_ResetConfig[ FLASH_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1RSTR;

    /*------------------------------------------------
    FLASH clocking bus source identifier
    ------------------------------------------------*/
     FLASH_SourceClock[ FLASH_RESOURCE_INDEX ] = Chimera::Clock::Bus::HCLK;
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_FLASH */
