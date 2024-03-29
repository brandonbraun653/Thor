/******************************************************************************
 *  File Name:
 *    version.hpp
 *
 *  Description:
 *    Low level driver versioning information for STM32F4 chips
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_STM32F4_VERSION_HPP
#define THOR_LLD_STM32F4_VERSION_HPP

/* STL Includes */
#include <string_view>

namespace Thor::LLD
{
  /**
   *  v2.4: Removed boost
   */
  static const std::string_view VersionSTM32F4 = "2.4";
}

#endif  /* !THOR_LLD_STM32F4_VERSION_HPP */
