/********************************************************************************
 *  File Name:
 *    thor_system.cpp
 *
 *  Description:
 *    Implements system interface functions for Thor
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */

/* Chimera Includes */

/* Thor Includes */
#include <Thor/preprocessor.hpp>
#include <Thor/headers.hpp>
#include <Thor/system.hpp>

namespace Thor
{
  namespace System
  {
    Identifier::Identifier()
    {
    }

    Identifier::~Identifier()
    {
    }

    uint32_t Identifier::deviceID()
    {
      return HAL_GetDEVID();
    }

    uint32_t Identifier::uniqueID()
    {
      /* Will likely need to switch this up as the unique ID is 96 bits */
      return 0u;
    }
  }
}