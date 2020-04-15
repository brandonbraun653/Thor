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
  static const std::string_view Version = "2.2#";
}

#endif  /* !THOR_HLD_VERSION_HPP */