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
  static std::array<Mock::DriverMock, NUM_CAN_PERIPHS> s_can_drivers;


  /*-------------------------------------------------------------------------------
  Mock Public Functions
  -------------------------------------------------------------------------------*/
  namespace Mock
  {
    static ModuleMock moduleMock;

    ModuleMock &getModuleMockObject()
    {
      return moduleMock;
    }

    DriverMock &getDriverMockObject( const size_t channel )
    {
      return s_can_drivers[ channel ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    // /*-------------------------------------------------
    // Mock behavior
    // -------------------------------------------------*/
    // Mock::getModuleMockObject().initialize();

    // /*-------------------------------------------------
    // Driver behavior
    // -------------------------------------------------*/
    // initializeRegisters();
    // initializeMapping();

    return Chimera::Status::OK;
  }

  Driver_rPtr getDriver( const Chimera::CAN::Channel channel )
  {
    // /*-------------------------------------------------
    // Mock behavior
    // -------------------------------------------------*/
    // Mock::getModuleMockObject().getDriver( static_cast<Chimera::CAN::Channel>( channel ) );

    // /*-------------------------------------------------
    // Driver behavior
    // -------------------------------------------------*/
    // if ( !( channel < NUM_CAN_PERIPHS ) )
    // {
    //   return nullptr;
    // }

    // s_can_drivers[ channel ].attach( PeripheralRegisterMaps[ channel ] );
    // return &s_can_drivers[ channel ];
    return nullptr;
  }


}    // namespace Thor::LLD::CAN

#endif /* THOR_LLD_CAN_MOCK */
