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

namespace Thor::LLD:DES
{
  static ModuleMock mockObj;

  ModuleMock &getMockObject()
  {
    return mockObj;
  }
  
  void getUniqueId( UniqueID &id )
  {
    return mockObj.getUniqueId( id );
  }

  size_t getFlashSize()
  {
    return mockObj.getFlashSize();
  }

  Chimera::System::Packaging getICPackaging()
  {
    return mockObj.getICPackaging();
  }
}    // namespace Thor::LLD::DES

#endif /* THOR_LLD_DES_MOCK */
