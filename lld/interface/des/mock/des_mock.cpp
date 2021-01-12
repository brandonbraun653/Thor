/********************************************************************************
 *  File Name:
 *    des_mock.cpp
 *
 *  Description:
 *    Implements the DES interface using GMock
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/des/des_intf.hpp>
#include <Thor/lld/interface/des/mock/des_mock.hpp>

#if defined( THOR_LLD_DES_MOCK )

namespace Thor::LLD::DES
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


  void getUniqueId( UniqueID &id )
  {
    ensureModuleMockExists();
    Mock::moduleMock->getUniqueId( id );
  }


  size_t getFlashSize()
  {
    return Mock::moduleMock->getFlashSize();
  }


  Chimera::System::Packaging getICPackaging()
  {
    return Mock::moduleMock->getICPackaging();
  }

}    // namespace Thor::LLD::DES

#endif /* THOR_LLD_DES_MOCK */
