/********************************************************************************
 *  File Name:
 *    can_mock.hpp
 *
 *  Description:
 *    Mock interface for CAN
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_LLD_CAN_MOCK_HPP
#define THOR_LLD_CAN_MOCK_HPP

/* STL Includes */
#include <memory>

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/can/can_types.hpp>
#include <Thor/lld/interface/can/mock/can_mock_variant.hpp>

#if defined( THOR_LLD_CAN_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

namespace Thor::LLD::CAN::Mock
{
  /*-------------------------------------------------------------------------------
  Mock Interfaces
  -------------------------------------------------------------------------------*/
  class IModule
  {
  public:
    virtual ~IModule()                                                                          = default;
    virtual Chimera::Status_t initialize()                                                      = 0;
    virtual Driver_rPtr getDriver( const Chimera::CAN::Channel )                                = 0;
    virtual bool isSupported( const Chimera::CAN::Channel )                                     = 0;
    virtual RIndex_t getResourceIndex( const std::uintptr_t )                                   = 0;
    virtual RIndex_t getResourceIndex( const Chimera::CAN::Channel )                            = 0;
    virtual Chimera::CAN::Channel getChannel( const std::uintptr_t )                            = 0;
    virtual bool attachDriverInstances( Driver *const, const size_t )                           = 0;
    virtual bool sortFiltersBySize( const MessageFilter *const, const uint8_t, uint8_t *const ) = 0;
  };

  class IDriver
  {
  public:
    virtual ~IDriver() = default;

    /*-------------------------------------------------------------------------------
    Configuration
    -------------------------------------------------------------------------------*/
    virtual void attach( RegisterMap *const peripheral ) = 0;
    virtual void enableClock() = 0;
    virtual void disableClock() = 0;
    virtual Chimera::Status_t configure( const Chimera::CAN::DriverConfig &cfg ) = 0;
    virtual Chimera::Status_t applyFilters( Thor::LLD::CAN::MessageFilter *const filterList, const size_t filterSize ) = 0;
    virtual Chimera::Status_t enableISRSignal( const Chimera::CAN::InterruptType signal ) = 0;
    virtual void disableISRSignal( const Chimera::CAN::InterruptType signal ) = 0;
    virtual void enterDebugMode( const Chimera::CAN::DebugMode mode ) = 0;
    virtual void exitDebugMode() = 0;

    /*-------------------------------------------------------------------------------
    Control
    -------------------------------------------------------------------------------*/
    virtual void flushTX() = 0;
    virtual void flushRX() = 0;

    /*-------------------------------------------------------------------------------
    Transmit & Receive Operations
    -------------------------------------------------------------------------------*/
    virtual Chimera::Status_t send( const Chimera::CAN::BasicFrame &frame ) = 0;
    virtual Chimera::Status_t receive( Chimera::CAN::BasicFrame &frame ) = 0;

    /*-------------------------------------------------------------------------------
    Asynchronous Operation
    -------------------------------------------------------------------------------*/
    virtual Chimera::Threading::BinarySemaphore *getISRSignal( Chimera::CAN::InterruptType signal ) = 0;
    virtual const ISREventContext *const getISRContext( const Chimera::CAN::InterruptType isr ) = 0;
    virtual void setISRHandled( const Chimera::CAN::InterruptType isr ) = 0;

    /*-------------------------------------------------------------------------------
    ISR Protection Mechanisms
    -------------------------------------------------------------------------------*/
    virtual void enterCriticalSection() = 0;
    virtual void exitCriticalSection() = 0;
  };


  /*-------------------------------------------------------------------------------
  Mock Classes
  -------------------------------------------------------------------------------*/
  class ModuleMock : public IModule
  {
  public:
    MOCK_METHOD( Chimera::Status_t, initialize, (), ( override ) );
    MOCK_METHOD( Driver_rPtr, getDriver, ( const Chimera::CAN::Channel ), ( override ) );
    MOCK_METHOD( bool, isSupported, ( const Chimera::CAN::Channel ), ( override ) );
    MOCK_METHOD( RIndex_t, getResourceIndex, ( const std::uintptr_t ), ( override ) );
    MOCK_METHOD( RIndex_t, getResourceIndex, ( const Chimera::CAN::Channel ), ( override ) );
    MOCK_METHOD( Chimera::CAN::Channel, getChannel, ( const std::uintptr_t ), ( override ) );
    MOCK_METHOD( bool, attachDriverInstances, ( Driver *const, const size_t ), ( override ) );
    MOCK_METHOD( bool, sortFiltersBySize, ( const MessageFilter *const, const uint8_t, uint8_t *const ), ( override ) );
  };

  class DriverMock : public IDriver
  {
  public:
    MOCK_METHOD( void, attach, ( RegisterMap *const ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, configure, ( const Chimera::CAN::DriverConfig & ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, applyFilters, ( Thor::LLD::CAN::MessageFilter *const, const size_t ),
                 ( override ) );
    MOCK_METHOD( Chimera::Status_t, enableISRSignal, ( const Chimera::CAN::InterruptType ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, send, ( const Chimera::CAN::BasicFrame & ), ( override ) );
    MOCK_METHOD( Chimera::Status_t, receive, ( Chimera::CAN::BasicFrame & ), ( override ) );
    MOCK_METHOD( Chimera::Threading::BinarySemaphore *, getISRSignal, ( Chimera::CAN::InterruptType ), ( override ) );
    MOCK_METHOD( const ISREventContext *const, getISRContext, ( const Chimera::CAN::InterruptType ), ( override ) );
    MOCK_METHOD( void, disableISRSignal, ( const Chimera::CAN::InterruptType ), ( override ) );
    MOCK_METHOD( void, enterDebugMode, ( const Chimera::CAN::DebugMode ), ( override ) );
    MOCK_METHOD( void, exitDebugMode, (), ( override ) );
    MOCK_METHOD( void, flushTX, (), ( override ) );
    MOCK_METHOD( void, flushRX, (), ( override ) );
    MOCK_METHOD( void, setISRHandled, ( const Chimera::CAN::InterruptType ), ( override ) );
    MOCK_METHOD( void, enterCriticalSection, (), ( override ) );
    MOCK_METHOD( void, exitCriticalSection, (), ( override ) );
    MOCK_METHOD( void, enableClock, (), ( override ) );
    MOCK_METHOD( void, disableClock, (), ( override ) );
  };


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  ModuleMock &getModuleMockObject();
  DriverMock &getDriverMockObject( const Chimera::CAN::Channel channel );

}    // namespace Thor::LLD::CAN::Mock

#endif /* THOR_LLD_CAN_MOCK */
#endif /* !THOR_LLD_CAN_MOCK_HPP */
