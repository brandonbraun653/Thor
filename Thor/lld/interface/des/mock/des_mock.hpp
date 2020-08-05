/********************************************************************************
 *  File Name:
 *    des_mock.hpp
 *
 *  Description:
 *    Mock interface for DES
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_MOCK_DES_HPP
#define THOR_LLD_MOCK_DES_HPP

/* STL Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/system>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/des/des_types.hpp>

#if defined( THOR_LLD_DES_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace Thor::LLD::DES::Mock
{
  /*-------------------------------------------------------------------------------
  Mock Interfaces
  -------------------------------------------------------------------------------*/
  /**
   *  Encapsulates the C-style interface to DES so that it can be
   *  mocked appropriately. Useless outside of testing purposes.
   */
  class IModule
  {
  public:
    virtual ~IModule() = default;

    virtual void initialize()                           = 0;
    virtual void getUniqueId( UniqueID &id )            = 0;
    virtual size_t getFlashSize()                       = 0;
    virtual Chimera::System::Packaging getICPackaging() = 0;
  };


  /*-------------------------------------------------------------------------------
  Mock Classes
  -------------------------------------------------------------------------------*/
  class ModuleMock : public IModule
  {
  public:
    MOCK_METHOD( void, initialize, (), (override) );
    MOCK_METHOD( void, getUniqueId, ( UniqueID & id ), ( override ) );
    MOCK_METHOD( size_t, getFlashSize, (), ( override ) );
    MOCK_METHOD( Chimera::System::Packaging, getICPackaging, (), ( override ) );
  };

  /**
   *  Gets the mock object for this module
   *
   *  @return ModuleMock&
   */
  ModuleMock &getMockObject();

}    // namespace Thor::LLD::DES

#endif /* THOR_LLD_DES_MOCK */
#endif /* !THOR_LLD_MOCK_DES_HPP */
