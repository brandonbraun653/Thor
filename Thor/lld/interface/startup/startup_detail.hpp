/********************************************************************************
 *  File Name:
 *    startup.hpp
 *
 *  Description:
 *    Includes chip level startup functions
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_STARTUP_DETAIL_HPP
#define THOR_STARTUP_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/startup/mock/startup_mock.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/startup/startup_stm32f4xxxx.hpp>
#else
#pragma message( "startup_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_STARTUP_DETAIL_HPP */
