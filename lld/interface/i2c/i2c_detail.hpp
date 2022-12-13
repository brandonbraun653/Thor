/******************************************************************************
 *  File Name:
 *    i2c_detail.hpp
 *
 *  Description:
 *    Includes the LLD specific headers for chip implementation details
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_I2C_INTF_DETAIL_HPP
#define THOR_LLD_I2C_INTF_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/i2c/mock/i2c_mock.hpp>
#include <Thor/lld/interface/i2c/mock/i2c_mock_variant.hpp>
#elif defined( TARGET_LLD_TEST )
#include <Thor/lld/interface/i2c/sim/i2c_sim_variant.hpp>
#include <Thor/lld/interface/i2c/sim/i2c_sim_types.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/i2c/hw_i2c_prj.hpp>
#include <Thor/lld/stm32f4x/i2c/hw_i2c_types.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/i2c/hw_i2c_prj.hpp>
#include <Thor/lld/stm32l4x/i2c/hw_i2c_types.hpp>
#else
#pragma message( "i2c_detail.hpp: Unknown target for LLD" )
#endif

#endif  /* !THOR_LLD_I2C_INTF_DETAIL_HPP */
