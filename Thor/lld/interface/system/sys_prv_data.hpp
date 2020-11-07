/********************************************************************************
 *  File Name:
 *    sys_prv_data.hpp
 *
 *  Description:
 *    Insert Description
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SYS_DATA_HPP
#define THOR_LLD_SYS_DATA_HPP

/* Thor Includes */
#include <Thor/lld/interface/system/sys_detail.hpp>

namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
  extern RegisterMap *SYSCFG1_PERIPH;

}  // namespace Thor::LLD::SYS

#endif  /* !THOR_LLD_SYS_DATA_HPP */
