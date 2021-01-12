/********************************************************************************
 *  File Name:
 *    sys_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SYS_INTF_DETAIL_HPP
#define THOR_LLD_SYS_INTF_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/system/mock/sys_mock.hpp>
#include <Thor/lld/interface/system/mock/sys_mock_variant.hpp>
#elif defined( TARGET_LLD_TEST )
#include <Thor/lld/interface/system/sim/sys_sim_variant.hpp>
#include <Thor/lld/interface/system/sim/sys_sim_types.hpp>
#elif defined( TARGET_STM32F4 )
#pragma message( "sys_detail.hpp not implemented for this processor")
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/system/hw_sys_prj.hpp>
#include <Thor/lld/stm32l4x/system/hw_sys_types.hpp>
#else
#pragma message( "sys_detail.hpp: Unknown target for LLD" )
#endif

#endif  /* !THOR_LLD_SYS_INTF_DETAIL_HPP */
