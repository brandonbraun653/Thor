/******************************************************************************
 *  File Name:
 *    hw_power_data.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/power>

#if defined( TARGET_STM32F4 ) && defined( THOR_PWR )

namespace Thor::LLD::PWR
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------*/
  RegisterMap *PWR_PERIPH = reinterpret_cast<RegisterMap *>( PWR_BASE_ADDR );

  /*-------------------------------------------------
  Configuration Maps
  -------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */
  } /* clang-format on */

  /*-------------------------------------------------
  Peripheral Resources
  -------------------------------------------------*/
  namespace Resource
  { /* clang-format off */
  } /* clang-format on */
}    // namespace Thor::LLD::PWR


#endif /* TARGET_STM32L4 && THOR_LLD_PWR */
