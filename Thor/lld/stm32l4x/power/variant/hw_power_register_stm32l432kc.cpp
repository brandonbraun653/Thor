/********************************************************************************
 *  File Name:
 *    hw_power_register_stm32l432kc.cpp
 *
 *  Description:
 *    POWER register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/power/hw_power_driver.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_mapping.hpp>
#include <Thor/lld/stm32l4x/power/variant/hw_power_register_stm32l432kc.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_PWR )

namespace Thor::LLD::PWR
{
#if defined( EMBEDDED )
  /*-------------------------------------------------
  Memory Mapped Structs to Peripherals
  -------------------------------------------------*/
  RegisterMap *POWER1_PERIPH = reinterpret_cast<RegisterMap *>( POWER1_BASE_ADDR );

  /*-------------------------------------------------
  Lookup Tables Defintions
  -------------------------------------------------*/
  Thor::LLD::RIndexMap InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( POWER1_PERIPH ), POWER1_RESOURCE_INDEX }
  };

#elif defined( _SIM )
  /*-------------------------------------------------
  Memory Mapped Structs to Virtual Peripherals
  -------------------------------------------------*/
  RegisterMap *POWER1_PERIPH = nullptr;

  /*-------------------------------------------------
  Lookup Tables Defintions
  -------------------------------------------------*/
  Thor::LLD::RIndexMap InstanceToResourceIndex;
#endif

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    POWER1_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( POWER1_PERIPH ), POWER1_RESOURCE_INDEX );
#endif
  }
}    // namespace Thor::LLD::PWR

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_power_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig PWR_ClockConfig[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];
  RegisterConfig PWR_ResetConfig[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];
  Chimera::Clock::Bus PWR_SourceClock[ Thor::LLD::PWR::NUM_POWER_PERIPHS ];

  PCC PWRLookup = { PWR_ClockConfig,
                    nullptr,
                    PWR_ResetConfig,
                    PWR_SourceClock,
                    &Thor::LLD::PWR::InstanceToResourceIndex,
                    Thor::LLD::PWR::NUM_POWER_PERIPHS };

  void PWRInit()
  {
    using namespace Thor::LLD::PWR;

    /*------------------------------------------------
    PWR clock enable register access lookup table
    ------------------------------------------------*/
    PWR_ClockConfig[ POWER1_RESOURCE_INDEX ].mask = APB1ENR1_PWREN;
    PWR_ClockConfig[ POWER1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->AHB1ENR;

    /*------------------------------------------------
    PWR reset register access lookup table
    ------------------------------------------------*/
    PWR_ResetConfig[ POWER1_RESOURCE_INDEX ].mask = APB1RSTR1_PWRRST;
    PWR_ResetConfig[ POWER1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;

    /*------------------------------------------------
    PWR clocking bus source identifier
    ------------------------------------------------*/
    PWR_SourceClock[ POWER1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_PWR */
