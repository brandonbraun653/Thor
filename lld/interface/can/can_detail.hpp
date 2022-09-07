/********************************************************************************
 *  File Name:
 *    can_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_CAN_INTF_DETAIL_HPP
#define THOR_LLD_CAN_INTF_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/can/mock/can_mock.hpp>
#include <Thor/lld/interface/can/mock/can_mock_variant.hpp>
#elif defined( TARGET_LLD_TEST )
#include <Thor/lld/interface/can/sim/can_sim_variant.hpp>
#include <Thor/lld/interface/can/sim/can_sim_types.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/can/hw_can_prj.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/can/hw_can_prj.hpp>
#else
#pragma message( "can_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_LLD_CAN_INTF_DETAIL_HPP */
