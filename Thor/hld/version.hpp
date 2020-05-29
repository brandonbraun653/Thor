/********************************************************************************
 *  File Name:
 *    version.hpp
 *
 *  Description:
 *    High level driver versioning information
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_VERSION_HPP
#define THOR_HLD_VERSION_HPP

/* STL Includes */
#include <string>

namespace Thor::HLD
{
  static constexpr std::string_view VersionString = "2.2.0";

  static constexpr size_t VersionMajor = 2;
  static constexpr size_t VersionMinor = 2;
  static constexpr size_t VersionPatch = 0;
}

#endif  /* !THOR_HLD_VERSION_HPP */