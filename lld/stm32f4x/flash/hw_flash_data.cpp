/********************************************************************************
 *  File Name:
 *    hw_flash_data.cpp
 *
 *  Description:
 *    Provides implementation details for private FLASH driver data
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/watchdog>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/flash>

#if defined( TARGET_STM32F4 ) && defined( THOR_FLASH )

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_FLASH_PERIPH_AVAILABLE )
  RegisterMap *FLASH_PERIPH = reinterpret_cast<RegisterMap *>( FLASH_BASE_ADDR );
#endif
}

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------------------------------------
  Configuration Maps
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */
  } /* clang-format on */


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  { /* clang-format off */
  } /* clang-format on */
}  // namespace Thor::LLD::FLASH

#endif /* TARGET_STM32L4 && THOR_LLD_FLASH */
