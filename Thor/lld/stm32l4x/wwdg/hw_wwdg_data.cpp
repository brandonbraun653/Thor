/********************************************************************************
 *  File Name:
 *    hw_wwdg_data.cpp
 *
 *  Description:
 *    Provides implementation details for private WWDG driver data
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/wwdg>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/wwdg/wwdg_prv_data.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_WWDG )

namespace Thor::LLD::WWDG
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
  RegisterMap *WWDG1_PERIPH = reinterpret_cast<RegisterMap *>( WWDG1_BASE_ADDR );
#endif

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
    LLD_CONST IRQn_Type IRQSignals[ NUM_WWDG_PERIPHS ] = {
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
      WWDG1_IRQn,
#endif
    };
  } /* clang-format on */
}  // namespace Thor::LLD::WWDG

#endif /* TARGET_STM32L4 && THOR_LLD_WWDG */
