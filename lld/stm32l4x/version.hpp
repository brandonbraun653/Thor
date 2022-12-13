/******************************************************************************
 *  File Name:
 *    version.hpp
 *
 *  Description:
 *    Low level driver versioning information for STM32L4 chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_STM32L4_VERSION_HPP
#define THOR_LLD_STM32L4_VERSION_HPP

/* STL Includes */
#include <string>

namespace Thor::LLD
{
  /**
   *  CHANGELOG:
   *
   *  v1.1: Upgraded the build system. Added many new drivers.
   *  v1.2: Removed boost
   */
  static const std::string_view VersionSTM32L4 = "1.2";
}    // namespace Thor::LLD

#endif /* !THOR_LLD_STM32L4_VERSION_HPP */
