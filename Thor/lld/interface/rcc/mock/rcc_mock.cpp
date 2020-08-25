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
#include <Thor/cfg>
#include <Thor/lld/interface/rcc/mock/rcc_mock.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>

#if defined( THOR_LLD_RCC_MOCK )

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Static Function Declaration
  -------------------------------------------------------------------------------*/
  static void ensureModuleMockExists();


  /*-------------------------------------------------------------------------------
  Mock Public Functions
  -------------------------------------------------------------------------------*/
  namespace Mock
  {
    static ModuleMock *moduleMock;
    static CoreClockMock *clockController;
    static PeripheralClockMock *periphController;

    ModuleMock &getMockObject()
    {
      ensureModuleMockExists();
      return ( *moduleMock );
    }

  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Static Function Definition
  -------------------------------------------------------------------------------*/
  static void ensureModuleMockExists()
  {
    if ( !Mock::moduleMock )
    {
      Mock::moduleMock = new Mock::ModuleMock();
    }
  }


  /*-------------------------------------------------------------------------------
  Mock C-Style RCC Interface
  -------------------------------------------------------------------------------*/
  void initialize()
  {
    ensureModuleMockExists();
    Mock::moduleMock->initialize();
  }


  Chimera::System::ResetEvent getResetReason()
  {
    ensureModuleMockExists();
    return Mock::moduleMock->getResetReason();
  }


  void clearResetReason()
  {
    ensureModuleMockExists();
    Mock::moduleMock->clearResetReason();
  }


  ICoreClock *getCoreClock()
  {
    if ( !Mock::clockController )
    {
      Mock::clockController = new Mock::CoreClockMock();
    }

    return Mock::clockController;
  }


  IPeripheralClock *getPeripheralClock()
  {
    if ( !Mock::periphController )
    {
      Mock::periphController = new Mock::PeripheralClockMock();
    }

    return Mock::periphController;
  }

}    // namespace Thor::LLD::RCC

#endif /* THOR_LLD_RCC_MOCK */
