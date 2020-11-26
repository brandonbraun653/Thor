/********************************************************************************
 *  File Name:
 *    hw_usb_data.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/usb>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usb/usb_prv_data.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_USB )

namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
#if defined( STM32_USB1_PERIPH_AVAILABLE )
  RegisterMap *USB1_PERIPH = reinterpret_cast<RegisterMap *>( USB1_BASE_ADDR );
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
    LLD_CONST IRQn_Type IRQSignals[ NUM_USB_PERIPHS ] = {
#if defined( STM32_USB1_PERIPH_AVAILABLE )
      OTG_FS_IRQn,
#endif
    };
  } /* clang-format on */
}    // namespace Thor::LLD::USB

#endif /* TARGET_STM32L4 && THOR_LLD_USB */
