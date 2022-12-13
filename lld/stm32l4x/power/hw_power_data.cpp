/******************************************************************************
 *  File Name:
 *    hw_power_data.cpp
 *
 *  Description:
 *    Provides implementation details for private PWR driver data
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/watchdog>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/power/power_detail.hpp>
#include <Thor/lld/interface/power/power_prv_data.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_PWR )

namespace Thor::LLD::PWR
{
  /*---------------------------------------------------------------------------
  Peripheral Memory Maps
  ---------------------------------------------------------------------------*/
#if defined( STM32_PWR_PERIPH_AVAILABLE )
  RegisterMap *PWR_PERIPH = reinterpret_cast<RegisterMap *>( PWR_BASE_ADDR );
#endif
}

namespace Thor::LLD::PWR
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
}  // namespace Thor::LLD::PWR

#endif /* TARGET_STM32L4 && THOR_LLD_PWR */
