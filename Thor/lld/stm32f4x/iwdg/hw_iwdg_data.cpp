/********************************************************************************
 *  File Name:
 *    hw_iwdg_data.cpp
 *
 *  Description:
 *    Provides implementation details for private IWDG driver data
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/watchdog>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/watchdog/watchdog_prv_data.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_IWDG )

namespace Thor::LLD::Watchdog
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
  IRegisterMap *IWDG1_PERIPH = reinterpret_cast<IRegisterMap *>( IWDG::IWDG1_BASE_ADDR );
#endif
}    // namespace Thor::LLD::Watchdog


namespace Thor::LLD::IWDG
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
}    // namespace Thor::LLD::IWDG

#endif /* TARGET_STM32L4 && THOR_LLD_IWDG */
