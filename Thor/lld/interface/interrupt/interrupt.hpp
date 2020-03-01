/********************************************************************************
 *  File Name:
 *    interrupt.hpp
 *
 *  Description:
 *    Common header for Thor Interrupt that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_INTERRUPT_CONFIG_HPP
#define THOR_INTERRUPT_CONFIG_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/interrupt/mock/interrupt_mock.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/drivers/f4/interrupt/hw_it_prj.hpp>
#else
#error Unknown Thor interrupt driver implementation
#endif /* THOR_CUSTOM_DRIVERS */

#endif