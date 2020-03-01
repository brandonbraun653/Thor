/********************************************************************************
 *  File Name:
 *    rcc_mock.cpp
 *
 *  Description:
 *    RCC Mock Driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */


/* Thor Includes */
#include <Thor/lld/interface/rcc/mock/rcc_mock.hpp>
#include <Thor/lld/interface/rcc/rcc_model.hpp>

namespace Thor::LLD::RCC 
{
  static SystemClockMock *clockController;
  static PeriphControllerMock *periphController;

  IClockTree *getSystemClockController()
  {
    if( !clockController )
    {
      clockController = new SystemClockMock();
    }

    return SystemClockMock;
  }

  IPeripheralController *getSystemPeripheralController()
  {
    if( !periphController )
    {
      periphController = new PeriphControllerMock();
    }

    return periphController;
  }

}