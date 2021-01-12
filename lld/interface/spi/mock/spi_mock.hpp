/********************************************************************************
 *  File Name:
 *    spi_mock.hpp
 *
 *  Description:
 *    Mock interface for SPI
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_MOCK_HPP
#define THOR_LLD_SPI_MOCK_HPP

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/spi/spi_intf.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>

#if defined( THOR_LLD_SPI_MOCK )
/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::SPI::Mock
{
  /*-------------------------------------------------------------------------------
  Mock Interfaces
  -------------------------------------------------------------------------------*/
  class IModule
  {
  public:
    virtual ~IModule()                                             = default;
    virtual Chimera::Status_t initialize()                         = 0;
    virtual bool isChannelSupported( const Chimera::SPI::Channel ) = 0;
    virtual IDriver_rPtr getDriver( const Chimera::SPI::Channel )  = 0;
    virtual size_t getResourceIndex( const Chimera::SPI::Channel ) = 0;
  };

  /*-------------------------------------------------------------------------------
  Mock Classes
  -------------------------------------------------------------------------------*/
  class ModuleMock : public IModule
  {
  public:
    MOCK_METHOD( Chimera::Status_t, initialize, (), ( override ) );
    MOCK_METHOD( bool, isChannelSupported, ( const Chimera::SPI::Channel ), ( override ) );
    MOCK_METHOD( IDriver_rPtr, getDriver, ( const Chimera::SPI::Channel ), ( override ) );
    MOCK_METHOD( size_t, getResourceIndex, ( const Chimera::SPI::Channel ), ( override ) );
  };

  class DriverMock : virtual public Thor::LLD::SPI::IDriver
  {
  public:
    MOCK_METHOD( Chimera::Status_t, attach, ( RegisterMap *const peripheral ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, reset, (), ( override ) );
    MOCK_METHOD( void, clockEnable, (), ( override ) );
    MOCK_METHOD( void, clockDisable, (), ( override ) );
    MOCK_METHOD( size_t, getErrorFlags, (), ( override ) );
    MOCK_METHOD( size_t, getStatusFlags, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, configure, ( const Chimera::SPI::DriverConfig & ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, registerConfig, ( Chimera::SPI::DriverConfig * ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, transfer, ( const void *const, void *const, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, transferIT, ( const void *const, void *const, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, transferDMA, ( const void *const, void *const, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, killTransfer, (), ( override ) );
    MOCK_METHOD( void, attachISRWakeup, ( Chimera::Threading::BinarySemaphore *const ), ( override ) );
    MOCK_METHOD( HWTransfer, getTransferBlock, (), ( override ) );
  };

  /*-------------------------------------------------------------------------------
  Mock Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const Chimera::SPI::Channel channel );

}    // namespace Thor::LLD::SPI::Mock

#endif /* THOR_LLD_SPI_MOCK */
#endif /* !THOR_LLD_SPI_MOCK_HPP */
