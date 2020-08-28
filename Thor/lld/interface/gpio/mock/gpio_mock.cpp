/********************************************************************************
 *  File Name:
 *    gpio_mock.cpp
 *
 *  Description:
 *    Mocks the GPIO driver interface
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Mock Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock.hpp>
#include <Thor/lld/interface/gpio/mock/gpio_mock_variant.hpp>

#if defined( THOR_LLD_GPIO_MOCK )

/* STL Includes */
#include <array>
#include <iostream>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static std::array<IDriver, NUM_GPIO_PERIPHS> s_gpio_drivers;


  /*-------------------------------------------------------------------------------
  Mock Public Functions
  -------------------------------------------------------------------------------*/
  namespace Mock
  {
    static ModuleMock moduleMock;

    ModuleMock &getMockObject()
    {
      return moduleMock;
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
    Mock::getMockObject().initialize();

    /*-------------------------------------------------
    Driver behavior
    -------------------------------------------------*/
    initializeRegisters();
    initializeMapping();

    return Chimera::Status::OK;
  }

  IDriver_rPtr getDriver( const size_t channel )
  {
    /*-------------------------------------------------
    Mock behavior
    -------------------------------------------------*/
    Mock::getMockObject().getDriver( channel );

    /*-------------------------------------------------
    Driver behavior
    -------------------------------------------------*/
    if ( !( channel < NUM_GPIO_PERIPHS ) )
    {
      return nullptr;
    }

    s_gpio_drivers[ channel ].attach( PeripheralRegisterMaps[ channel ] );
    return &s_gpio_drivers[ channel ];
  }

  size_t availableChannels()
  {
    /*-------------------------------------------------
    Mock behavior
    -------------------------------------------------*/
    Mock::getMockObject().availableChannels();

    /*-------------------------------------------------
    Driver behavior
    -------------------------------------------------*/
    return NUM_GPIO_PERIPHS;
  }
}    // namespace Thor::LLD::GPIO

#endif /* THOR_LLD_GPIO_MOCK */
