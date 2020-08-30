/********************************************************************************
 *  File Name:
 *    usart_mock.cpp
 *
 *  Description:
 *    Mocks the USART driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/mock/usart_mock.hpp>
#include <Thor/lld/interface/usart/mock/usart_mock_variant.hpp>

#if defined( THOR_LLD_USART_MOCK )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static std::array<Mock::DriverMock, NUM_SPI_PERIPHS> s_usart_drivers;

  static const std::array<size_t, static_cast<size_t>( Chimera::Serial::Channel::NUM_OPTIONS )> s_resource_index = {
    SPI1_RESOURCE_INDEX,
    SPI2_RESOURCE_INDEX,
    SPI3_RESOURCE_INDEX,
    SPI4_RESOURCE_INDEX,
    INVALID_RESOURCE_INDEX,
    INVALID_RESOURCE_INDEX
  };


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
      return s_usart_drivers[ channel ];
    }
  }    // namespace Mock


  /*-------------------------------------------------------------------------------
  Mock C-Style Interface
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Mock behavior
    -------------------------------------------------*/
    Mock::getModuleMockObject().initialize();

    /*-------------------------------------------------
    Driver behavior
    -------------------------------------------------*/
    initializeRegisters();
    initializeMapping();

    return Chimera::Status::OK;
  }


  bool isChannelSupported( const Chimera::Serial::Channel channel )
  {
    /*-------------------------------------------------
    Mock behavior
    -------------------------------------------------*/
    Mock::getModuleMockObject().isChannelSupported( channel );

    /*-------------------------------------------------
    Driver behavior
    -------------------------------------------------*/
    return ( static_cast<size_t>( channel ) < NUM_SPI_PERIPHS );
  }


  IDriver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    /*-------------------------------------------------
    Mock behavior
    -------------------------------------------------*/
    Mock::getModuleMockObject().getDriver( channel );

    /*-------------------------------------------------
    Driver behavior
    -------------------------------------------------*/
    if ( !isChannelSupported( channel ) )
    {
      return nullptr;
    }

    return nullptr;
  }


  RIndexType getResourceIndex( const Chimera::Serial::Channel channel )
  {
    /*-------------------------------------------------
    Mock behavior
    -------------------------------------------------*/
    Mock::getModuleMockObject().getResourceIndex( channel );

    /*-------------------------------------------------
    Driver behavior
    -------------------------------------------------*/
    if( channel < Chimera::Serial::Channel::NUM_OPTIONS )
    {
      return s_resource_index[ static_cast<size_t>( channel ) ];
    }
    else
    {
      return INVALID_RESOURCE_INDEX;
    }
  }


  RIndexType getResourceIndex( void *instance )
  {
    return INVALID_RESOURCE_INDEX;
  }

}    // namespace Thor::LLD::USART

#endif /* THOR_LLD_USART_MOCK */
