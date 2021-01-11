/********************************************************************************
 *  File Name:
 *    hw_flash_register_stm32f446kc.cpp
 *
 *  Description:
 *    FLASH register definitions for the STM32F446KC series chips.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/clock>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/flash/flash_intf.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_driver.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_mapping.hpp>
#include <Thor/lld/stm32f4x/flash/variant/hw_flash_register_stm32f446re.hpp>
#include <Thor/lld/stm32f4x/flash/hw_flash_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32F446xx ) && defined( THOR_LLD_FLASH )

namespace Thor::LLD::FLASH
{
#if defined( EMBEDDED )
  /*-------------------------------------------------
  Memory Mapped Structs to Peripherals
  -------------------------------------------------*/
  RegisterMap *FLASH_PERIPH = reinterpret_cast<RegisterMap *>( FLASH_BASE_ADDR );

#endif
}    // namespace Thor::LLD::FLASH

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_flash_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig FLASH_ClockConfig[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
  RegisterConfig FLASH_ResetConfig[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];
  Chimera::Clock::Bus FLASH_SourceClock[ Thor::LLD::FLASH::NUM_FLASH_PERIPHS ];

  PCC FLASHLookup = { FLASH_ClockConfig,
                      nullptr,
                      FLASH_ResetConfig,
                      FLASH_SourceClock,
                      Thor::LLD::FLASH::NUM_FLASH_PERIPHS,
                      Thor::LLD::FLASH::getResourceIndex };

  void FLASHInit()
  {
    using namespace Thor::LLD::FLASH;

    /*------------------------------------------------
    FLASH clock enable register access lookup table
    ------------------------------------------------*/
    FLASH_ClockConfig[ FLASH_RESOURCE_INDEX ].mask = CKGATENR_FLITF_CKEN;
    FLASH_ClockConfig[ FLASH_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->CKGATENR;

    /*------------------------------------------------
    FLASH reset register access lookup table
    ------------------------------------------------*/
    FLASH_ResetConfig[ FLASH_RESOURCE_INDEX ].mask = 0;
    FLASH_ResetConfig[ FLASH_RESOURCE_INDEX ].reg  = nullptr;

    /*------------------------------------------------
    FLASH clocking bus source identifier
    ------------------------------------------------*/
    FLASH_SourceClock[ FLASH_RESOURCE_INDEX ] = Chimera::Clock::Bus::AHB;
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32F446xx && THOR_LLD_FLASH */
