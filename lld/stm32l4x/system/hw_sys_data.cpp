/********************************************************************************
 *  File Name:
 *    hw_sys_data.cpp
 *
 *  Description:
 *    Insert Description
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/system/sys_prv_data.hpp>

namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
  RegisterMap *SYSCFG1_PERIPH = reinterpret_cast<RegisterMap *>( SYSCFG_BASE_ADDR );

}  // namespace Thor::LLD::SYS
