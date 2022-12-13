/******************************************************************************
 *  File Name:
 *    version.hpp
 *
 *  Description:
 *    High level driver versioning information
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HLD_VERSION_HPP
#define THOR_HLD_VERSION_HPP

/* STL Includes */
#include <string>

namespace Thor::HLD
{
  /**
   *  CHANGELOG:
   *  v2.5: Cleaned up the HLD
   *  v2.4: Removed boost
   *  v2.3: Upgraded the build system. Added many new drivers.
   */
  static constexpr std::string_view VersionString = "2.5.0";

  static constexpr size_t VersionMajor = 2;
  static constexpr size_t VersionMinor = 3;
  static constexpr size_t VersionPatch = 0;
}    // namespace Thor::HLD

#endif /* !THOR_HLD_VERSION_HPP */