/******************************************************************************
 *  File Name:
 *    hw_des_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_DES_PROJECT_HPP
#define THOR_LLD_DES_PROJECT_HPP

#if defined( STM32L432xx )
#include <Thor/lld/stm32l4x/des/variant/hw_des_register_stm32l432xx.hpp>
#else
#pragma message( "Unknown L4 chip for DES" )
#endif

#endif  /* !THOR_LLD_DES_PROJECT_HPP */
