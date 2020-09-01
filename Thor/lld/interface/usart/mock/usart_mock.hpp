/********************************************************************************
 *  File Name:
 *    usart_mock.hpp
 *
 *  Description:
 *    Mock interface for USART
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_USART_MOCK_HPP
#define THOR_LLD_USART_MOCK_HPP

/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_types.hpp>

#if defined( THOR_LLD_USART_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::USART::Mock
{
  /*-------------------------------------------------------------------------------
  Mock Interfaces
  -------------------------------------------------------------------------------*/
  class IModule
  {
  public:
    virtual ~IModule()                                                    = default;
    virtual Chimera::Status_t initialize()                                = 0;
    virtual bool isChannelSupported( const Chimera::Serial::Channel )     = 0;
    virtual IDriver_rPtr getDriver( const Chimera::Serial::Channel )      = 0;
    virtual RIndex_t getResourceIndex( const Chimera::Serial::Channel ) = 0;
    virtual RIndex_t getResourceIndex( void * )                         = 0;
  };


  /*-------------------------------------------------------------------------------
  Mock Classes
  -------------------------------------------------------------------------------*/
  class ModuleMock : public IModule
  {
  public:
    MOCK_METHOD( Chimera::Status_t, initialize, (), ( override ) );
    MOCK_METHOD( bool, isChannelSupported, ( const Chimera::Serial::Channel ), ( override ) );
    MOCK_METHOD( IDriver_rPtr, getDriver, ( const Chimera::Serial::Channel ), ( override ) );
    MOCK_METHOD( RIndex_t, getResourceIndex, ( const Chimera::Serial::Channel ), ( override ) );
    MOCK_METHOD( RIndex_t, getResourceIndex, ( void * ), ( override ) );
  };

  class DriverMock : virtual public Thor::LLD::USART::IDriver
  {
  public:
    MOCK_METHOD( Chimera::Status_t, init, ( const Thor::LLD::Serial::Config & ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, deinit, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, reset, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, transmit, ( const uint8_t *const, const size_t, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, receive, ( uint8_t *const, const size_t, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, txTransferStatus, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, rxTransferStatus, (), ( override ) );
    MOCK_METHOD( uint32_t, getFlags, (), ( override ) );
    MOCK_METHOD( void, clearFlags, ( const uint32_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, enableIT, ( const Chimera::Hardware::SubPeripheral ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, disableIT, ( const Chimera::Hardware::SubPeripheral ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, transmitIT, ( const uint8_t *const, const size_t, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, receiveIT, ( uint8_t *const, const size_t, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, initDMA, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, deinitDMA, (), ( override ) );
    MOCK_METHOD( Chimera::Status_t, enableDMA_IT, ( const Chimera::Hardware::SubPeripheral ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, disableDMA_IT, ( const Chimera::Hardware::SubPeripheral ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, transmitDMA, ( const void *const, const size_t, const size_t ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, receiveDMA, ( void *const, const size_t, const size_t ), ( override ) );
    MOCK_METHOD( void, killTransmit, (), ( override ) );
    MOCK_METHOD( void, killReceive, (), ( override ) );
    MOCK_METHOD( void, attachISRWakeup, ( Chimera::Threading::BinarySemaphore *const wakeup ), ( override ) );
    MOCK_METHOD( ::Thor::LLD::Serial::CDTCB, getTCB_TX, (), ( override ) );
    MOCK_METHOD( ::Thor::LLD::Serial::MDTCB, getTCB_RX, (), ( override ) );
    MOCK_METHOD( ::Thor::LLD::Serial::Config, getConfiguration, (), ( override ) );
  };


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const size_t channel );

}    // namespace Thor::LLD::USART::Mock

#endif /* THOR_LLD_USART_MOCK */
#endif /* !THOR_LLD_USART_MOCK_HPP */
