/******************************************************************************
 *  File Name:
 *    hld_timer_core.cpp
 *
 *  Description:
 *    Thor implementation of the Chimera timer core driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/timer>


namespace Chimera::Timer
{
  Core::Core()
  {
  }


  Core::~Core()
  {
  }


  Chimera::Status_t Core::initCore( const CoreConfig &cfg )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Core::startCounter()
  {

  }


  void Core::stopCounter()
  {

  }

}    // namespace Chimera::Timer::PWM
