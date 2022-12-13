/******************************************************************************
 *  File Name:
 *    startup_detail.hpp
 *
 *  Description:
 *    Includes chip level startup functions
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_STARTUP_DETAIL_HPP
#define THOR_STARTUP_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/startup/mock/startup_mock.hpp>
#elif defined( TARGET_STM32L4 ) || defined( TARGET_STM32F4 )
#include <Thor/lld/common/cortex-m4/cm4_startup_reset.hpp>
#else
#pragma message( "startup_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_STARTUP_DETAIL_HPP */
