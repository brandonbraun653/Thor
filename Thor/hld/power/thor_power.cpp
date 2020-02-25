/********************************************************************************
 *   File Name:
 *    power.hpp
 *
 *   Description:
 *    Models the system power interface 
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/
 
/* Chimera Includes */
#include <Chimera/power>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/power.hpp>

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