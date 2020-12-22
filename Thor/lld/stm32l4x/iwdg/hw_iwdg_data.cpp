/********************************************************************************
 *  File Name:
 *    hw_iwdg_data.cpp
 *
 *  Description:
 *    Provides implementation details for private IWDG driver data
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/iwdg>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/iwdg/iwdg_prv_data.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_IWDG )

namespace Thor::LLD::IWDG
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
  RegisterMap *IWDG1_PERIPH = reinterpret_cast<RegisterMap *>( IWDG1_BASE_ADDR );
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
    LLD_CONST IRQn_Type IRQSignals[ NUM_IWDG_PERIPHS ] = {
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
      IWDG1_IRQn,
#endif
    };
  } /* clang-format on */
}  // namespace Thor::LLD::IWDG

#endif /* TARGET_STM32L4 && THOR_LLD_IWDG */
