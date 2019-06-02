/********************************************************************************
 *  File Name:
 *    Ethernet.hpp
 *  
 *  Description:
 *    Common header for Thor ethernet that configures the driver based on which
 *    chip family is being compiled against.
 *  
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_ETHERNET_CONFIG_HPP
#define THOR_ETHERNET_CONFIG_HPP

#include <Thor/preprocessor.hpp>

#if defined( STM32F4 )

#endif /* STM32F4 */

#if defined( STM32F7 )

#endif /* STM32F7 */

#endif /* !THOR_ETHERNET_CONFIG_HPP */
