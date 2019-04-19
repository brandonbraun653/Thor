/********************************************************************************
 *   File Name:
 *    power.hpp
 *
 *   Description:
 *    Models the system power interface 
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/power.hpp>

/* Chimera Includes */
#include <Chimera/types/power_types.hpp>

namespace Thor
{
  namespace Power
  {
    SystemPower::SystemPower()
    {
      
    }

    SystemPower::~SystemPower()
    {
      
    }

    float SystemPower::systemVCC()
    {
      return 3.30f;
    }

  }    // namespace Power
}