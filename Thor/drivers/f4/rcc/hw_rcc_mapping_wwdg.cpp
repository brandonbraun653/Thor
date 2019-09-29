/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping_wwdg.cpp
 *
 *   Description:
 *    RCC configuration maps for the WWDG peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_mapping.hpp>

namespace Thor::Driver::RCC::LookupTables
{
/*------------------------------------------------
WWDG Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )
  /**
   *  WWDG clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_wwdg_mapping.hpp
   */
  const RegisterConfig WWDG_ClockConfig[ wwdgTableSize ] = {
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1ENR ) ),
      APB1ENR_WWDGEN }
  };

  /**
   *  WWDG low power clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_wwdg_mapping.hpp
   */
  const RegisterConfig WWDG_ClockConfigLP[ wwdgTableSize ] = {
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1LPENR ) ),
      APB1LPENR_WWDGLPEN }
  };

  /**
   *  WWDG reset register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_wwdg_mapping.hpp
   */
  const RegisterConfig WWDG_ResetConfig[ wwdgTableSize ] = {
    { reinterpret_cast<decltype( RegisterConfig::reg )>( RCC_BASE_ADDR + offsetof( RegisterMap, APB1RSTR ) ),
      APB1RSTR_WWDGRST }
  };

  /**
   *  WWDG clocking bus source identifier
   *
   *  @note Indexing must match the lookup table in hw_wwdg_mapping.hpp
   */
  const Configuration::ClockType_t WWDG_SourceClock[ wwdgTableSize ] = {
    Configuration::ClockType::HCLK
  };

  const PCC WWDGLookup = {
    WWDG_ClockConfig, WWDG_ClockConfigLP, WWDG_ResetConfig, WWDG_SourceClock, &Thor::Driver::WWDG::InstanceToResourceIndex,
    wwdgTableSize
  };

#endif /* TARGET_STM32F4 && THOR_DRIVER_WWDG */
}    // namespace Thor::Driver::RCC::LookupTables
