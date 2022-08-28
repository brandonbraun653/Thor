/********************************************************************************
 *  File Name:
 *    hw_des_register_stm32l432xx.cpp
 *
 *  Description:
 *    Register definitions for DES on STM32L432
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/des/hw_des_mapping.hpp>
#include <Thor/lld/stm32l4x/des/hw_des_prj.hpp>
#include <Thor/lld/stm32l4x/des/hw_des_types.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_DES ) && defined( STM32L432xx )

namespace Thor::LLD::DES
{
#if defined( EMBEDDED )
  UDIDRegisterMap *UDIDR = reinterpret_cast<UDIDRegisterMap *>( UDIDR_BASE_ADDRESS );
  FSRegisterMap *FSDR    = reinterpret_cast<FSRegisterMap *>( FSDR_BASE_ADDRESS );
  PDRegisterMap *PDR     = reinterpret_cast<PDRegisterMap *>( PDR_BASE_ADDRESS );
#endif
}    // namespace Thor::LLD::DES

#endif /* TARGET_STM32L4 && THOR_LLD_DES && STM32L432xx */
