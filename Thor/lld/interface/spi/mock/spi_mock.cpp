/********************************************************************************
 *  File Name:
 *    spi_mock.cpp
 *
 *  Description:
 *    Mocks the SPI driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <array>

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/spi/spi_intf.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>
#include <Thor/lld/interface/spi/mock/spi_mock.hpp>
#include <Thor/lld/interface/spi/mock/spi_mock_variant.hpp>

#if defined( THOR_LLD_SPI_MOCK )

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static std::array<Mock::DriverMock, NUM_SPI_PERIPHS> s_spi_drivers;

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
      return s_spi_drivers[ channel ];
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


  bool isChannelSupported( const Chimera::SPI::Channel channel )
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


  IDriver_rPtr getDriver( const Chimera::SPI::Channel channel )
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

    size_t ch = static_cast<size_t>( channel );
    s_spi_drivers[ ch ].attach( PeripheralRegisterMaps[ ch ] );
    return &s_spi_drivers[ ch ];
  }
}    // namespace Thor::LLD::SPI

#endif /* THOR_LLD_SPI_MOCK */
