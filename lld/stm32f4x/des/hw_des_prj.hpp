/********************************************************************************
 *  File Name:
 *    hw_des_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_DES_PROJECT_HPP
#define THOR_LLD_DES_PROJECT_HPP

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/des/variant/hw_des_register_stm32f446xx.hpp>
#else
#pragma message( "Unknown F4 chip for DES" )
#endif

#endif  /* !THOR_LLD_DES_PROJECT_HPP */
