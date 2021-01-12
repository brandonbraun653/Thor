/********************************************************************************
 *  File Name:
 *    watchdog_detail.hpp
 *
 *  Description:
 *    Watchdog includes
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_WATCHDOG_DETAIL_HPP
#define THOR_WATCHDOG_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/watchdog/mock/watchdog_mock.hpp>
#include <Thor/lld/interface/watchdog/mock/watchdog_mock_variant.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_prj.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_types.hpp>

#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_prj.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_types.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/wwdg/hw_wwdg_prj.hpp>
#include <Thor/lld/stm32l4x/wwdg/hw_wwdg_types.hpp>

#include <Thor/lld/stm32l4x/iwdg/hw_iwdg_prj.hpp>
#include <Thor/lld/stm32l4x/iwdg/hw_iwdg_types.hpp>
#else
#pragma message( "watchdog_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_WATCHDOG_DETAIL_HPP */
