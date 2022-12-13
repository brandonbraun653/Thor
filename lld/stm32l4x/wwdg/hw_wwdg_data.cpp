/******************************************************************************
 *  File Name:
 *    hw_wwdg_data.cpp
 *
 *  Description:
 *    Provides implementation details for private WWDG driver data
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/watchdog>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/watchdog/watchdog_prv_data.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_WWDG )

namespace Thor::LLD::Watchdog
{
  /*---------------------------------------------------------------------------
  Peripheral Memory Maps
  ---------------------------------------------------------------------------*/
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
  WRegisterMap *WWDG1_PERIPH = reinterpret_cast<WRegisterMap *>( WWDG::WWDG1_BASE_ADDR );
#endif
}

namespace Thor::LLD::WWDG
{
  /*---------------------------------------------------------------------------
  Configuration Maps
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */
  } /* clang-format on */


  /*---------------------------------------------------------------------------
  Peripheral Resources
  ---------------------------------------------------------------------------*/
  namespace Resource
  { /* clang-format off */
  } /* clang-format on */
}  // namespace Thor::LLD::WWDG

#endif /* TARGET_STM32L4 && THOR_LLD_WWDG */
