/********************************************************************************
 *  File Name:
 *    can_mock.cpp
 *
 *  Description:
 *    Mocks the CAN driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_types.hpp>
#include <Thor/lld/interface/can/mock/can_mock.hpp>
#include <Thor/lld/interface/can/mock/can_mock_variant.hpp>

#if defined( THOR_LLD_CAN_MOCK )

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static std::array<Mock::NiceDriverMock, NUM_CAN_PERIPHS> s_mock_drivers;
  static std::array<Driver, NUM_CAN_PERIPHS> s_can_drivers;


  /*-------------------------------------------------------------------------------
  Mock Public Functions
  -------------------------------------------------------------------------------*/
  namespace Mock
  {
    static NiceModuleMock moduleMock;

    NiceModuleMock &getModuleMockObject()
    {
      return moduleMock;
    }

    NiceDriverMock &getDriverMockObject( const Chimera::CAN::Channel channel )
    {
      return s_mock_drivers[ static_cast<size_t>( channel ) ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Driver behavior
    -------------------------------------------------*/

    initializeMapping();

    /*-------------------------------------------------
    Mock behavior
    -------------------------------------------------*/
    return Mock::getModuleMockObject().initialize();;
  }


  Driver_rPtr getDriver( const Chimera::CAN::Channel channel )
  {
    Mock::getModuleMockObject().getDriver( static_cast<Chimera::CAN::Channel>( channel ) );

    if( static_cast<size_t>( channel ) < NUM_CAN_PERIPHS )
    {
      return &s_can_drivers[ static_cast<size_t>( channel ) ];
    }
    else
    {
      return nullptr;
    }
  }


  RIndex_t getResourceIndex( const Chimera::CAN::Channel channel )
  {
    return Mock::getModuleMockObject().getResourceIndex( channel );
  }

  /*-------------------------------------------------------------------------------
  Mocked Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr ), mResourceIndex( 0 )
  {
  }


  Driver::~Driver()
  {
  }

  void Driver::attach( RegisterMap *const peripheral )
  {
  }


  void Driver::enableClock()
  {
  }


  void Driver::disableClock()
  {
  }


  Chimera::Status_t Driver::configure( const Chimera::CAN::DriverConfig &cfg )
  {
    return Mock::getDriverMockObject( TestChannel ).configure( cfg );
  }


  Chimera::Status_t Driver::applyFilters( MessageFilter *const filterList, const size_t filterSize )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::enableISRSignal( const Chimera::CAN::InterruptType signal )
  {
    return Chimera::Status::OK;
  }


  void Driver::disableISRSignal( const Chimera::CAN::InterruptType signal )
  {
  }


  void Driver::enterDebugMode( const Chimera::CAN::DebugMode mode )
  {
  }


  void Driver::exitDebugMode()
  {
  }


  Chimera::Status_t Driver::send( const Chimera::CAN::BasicFrame &frame )
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::receive( Chimera::CAN::BasicFrame &frame )
  {
    return Chimera::Status::OK;
  }


  void Driver::flushTX()
  {
  }


  void Driver::flushRX()
  {
  }


  Chimera::Threading::BinarySemaphore *Driver::getISRSignal( Chimera::CAN::InterruptType signal )
  {
    return nullptr;
  }


  const ISREventContext *const Driver::getISRContext( const Chimera::CAN::InterruptType isr )
  {
    return nullptr;
  }


  void Driver::setISRHandled( const Chimera::CAN::InterruptType isr )
  {
  }

}    // namespace Thor::LLD::CAN

#endif /* THOR_LLD_CAN_MOCK */
