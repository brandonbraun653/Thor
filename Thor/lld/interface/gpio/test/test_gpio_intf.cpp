/********************************************************************************
 *  File Name:
 *    test_gpio_intf.cpp
 *
 *  Description:
 *    Tests the interface layer for the low level GPIO drivers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/


/* Chimera Includes */
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_prv_data.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_GPIO )

/* GTest Includes */
#include <gtest/gtest.h>

namespace Thor::LLD::GPIO
{
  static Driver fakeDriverList[ NUM_GPIO_PERIPHS ];

  /*-------------------------------------------------------------------------------
  FUT: isSupported()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_GPIO_DriverSupport, InvalidInputs )
  {
    EXPECT_EQ( false, isSupported( Chimera::GPIO::Port::NUM_OPTIONS, 0 ) );
    EXPECT_EQ( false, isSupported( Chimera::GPIO::Port::PORTA, std::numeric_limits<Chimera::GPIO::Pin>::max() ) );
  }

  TEST( Normal_GPIO_DriverSupport, ValidInputs )
  {
    /*-------------------------------------------------
    This section uses hard coded values defined in the
    gpio_sim_variant.cpp file.
    -------------------------------------------------*/
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 0 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 1 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 2 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 3 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 4 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 5 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 6 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 7 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 8 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 9 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 10 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 11 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 12 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 13 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 14 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTA, 15 ) );

    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTB, 0 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTB, 1 ) );
    EXPECT_EQ( false, isSupported( Chimera::GPIO::Port::PORTB, 2 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTB, 3 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTB, 4 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTB, 5 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTB, 6 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTB, 7 ) );
    EXPECT_EQ( false, isSupported( Chimera::GPIO::Port::PORTB, 8 ) );
    EXPECT_EQ( false, isSupported( Chimera::GPIO::Port::PORTB, 9 ) );
    EXPECT_EQ( false, isSupported( Chimera::GPIO::Port::PORTB, 10 ) );

    EXPECT_EQ( false, isSupported( Chimera::GPIO::Port::PORTC, 0 ) );
    EXPECT_EQ( false, isSupported( Chimera::GPIO::Port::PORTC, 1 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTC, 14 ) );
    EXPECT_EQ( true, isSupported( Chimera::GPIO::Port::PORTC, 15 ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getPinResourceIndex()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_GPIO_PinResourceIndex, InvalidInputs )
  {
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getPinResourceIndex( Chimera::GPIO::Port::NUM_OPTIONS, 0 ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getPinResourceIndex( Chimera::GPIO::Port::PORTA, std::numeric_limits<Chimera::GPIO::Pin>::max() ) );
  }

  TEST( Normal_GPIO_PinResourceIndex, ValidInputs )
  {
    EXPECT_EQ( 0, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 0 ) );
    EXPECT_EQ( 1, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 1 ) );
    EXPECT_EQ( 2, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 2 ) );
    EXPECT_EQ( 3, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 3 ) );
    EXPECT_EQ( 4, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 4 ) );
    EXPECT_EQ( 5, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 5 ) );
    EXPECT_EQ( 6, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 6 ) );
    EXPECT_EQ( 7, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 7 ) );
    EXPECT_EQ( 8, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 8 ) );
    EXPECT_EQ( 9, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 9 ) );
    EXPECT_EQ( 10, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 10 ) );
    EXPECT_EQ( 11, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 11 ) );
    EXPECT_EQ( 12, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 12 ) );
    EXPECT_EQ( 13, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 13 ) );
    EXPECT_EQ( 14, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 14 ) );
    EXPECT_EQ( 15, getPinResourceIndex( Chimera::GPIO::Port::PORTA, 15 ) );

    EXPECT_EQ( 16, getPinResourceIndex( Chimera::GPIO::Port::PORTB, 0 ) );
    EXPECT_EQ( 17, getPinResourceIndex( Chimera::GPIO::Port::PORTB, 1 ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getPinResourceIndex( Chimera::GPIO::Port::PORTB, 2 ) );
    EXPECT_EQ( 18, getPinResourceIndex( Chimera::GPIO::Port::PORTB, 3 ) );
    EXPECT_EQ( 19, getPinResourceIndex( Chimera::GPIO::Port::PORTB, 4 ) );
    EXPECT_EQ( 20, getPinResourceIndex( Chimera::GPIO::Port::PORTB, 5 ) );
    EXPECT_EQ( 21, getPinResourceIndex( Chimera::GPIO::Port::PORTB, 6 ) );
    EXPECT_EQ( 22, getPinResourceIndex( Chimera::GPIO::Port::PORTB, 7 ) );

    EXPECT_EQ( INVALID_RESOURCE_INDEX, getPinResourceIndex( Chimera::GPIO::Port::PORTC, 0 ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getPinResourceIndex( Chimera::GPIO::Port::PORTC, 1 ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getPinResourceIndex( Chimera::GPIO::Port::PORTC, 2 ) );
    EXPECT_EQ( 23, getPinResourceIndex( Chimera::GPIO::Port::PORTC, 14 ) );
    EXPECT_EQ( 24, getPinResourceIndex( Chimera::GPIO::Port::PORTC, 15 ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getResourceIndex()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_GPIO_ResourceIndex, InvalidInputs )
  {
    // Variant 1: Peripheral Address
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( nullptr ) ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( std::numeric_limits<std::uintptr_t>::min() ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( std::numeric_limits<std::uintptr_t>::max() ) );

    // Variant 2: Chimera Ports
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::GPIO::Port::NUM_OPTIONS ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::GPIO::Port::UNKNOWN_PORT ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::GPIO::Port::PORTD ) );
  }

  TEST( Normal_GPIO_ResourceIndex, ValidInputs )
  {
    // Variant 1: Peripheral Address
    EXPECT_EQ( GPIOA_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ) ) );
    EXPECT_EQ( GPIOB_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ) ) );
    EXPECT_EQ( GPIOC_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ) ) );

    // Variant 2: Chimera Ports
    EXPECT_EQ( GPIOA_RESOURCE_INDEX, getResourceIndex( Chimera::GPIO::Port::PORTA ) );
    EXPECT_EQ( GPIOB_RESOURCE_INDEX, getResourceIndex( Chimera::GPIO::Port::PORTB ) );
    EXPECT_EQ( GPIOC_RESOURCE_INDEX, getResourceIndex( Chimera::GPIO::Port::PORTC ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getPinAttributes()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_GPIO_PinAttributes, InvalidInputs )
  {
    EXPECT_EQ( nullptr, getPinAttributes( Chimera::GPIO::Port::NUM_OPTIONS, 0 ) );
    EXPECT_EQ( nullptr, getPinAttributes( Chimera::GPIO::Port::PORTI, 0 ) );
    EXPECT_EQ( nullptr, getPinAttributes( Chimera::GPIO::Port::PORTA, std::numeric_limits<Chimera::GPIO::Pin>::max() ) );
  }

  TEST( Normal_GPIO_PinAttributes, ValidInputs )
  {
    static constexpr size_t a = static_cast<size_t>( Chimera::GPIO::Port::PORTA );
    static constexpr size_t b = static_cast<size_t>( Chimera::GPIO::Port::PORTB );
    static constexpr size_t c = static_cast<size_t>( Chimera::GPIO::Port::PORTC );


    const auto tmp1 = &( prjPortAttributes[ a ].pins[ 0 ] );
    EXPECT_EQ( tmp1, getPinAttributes( Chimera::GPIO::Port::PORTA, 0 ) );

    // This jumps over the missing pin 2
    const auto tmp2 = &( prjPortAttributes[ b ].pins[ 2 ] );
    EXPECT_EQ( tmp2, getPinAttributes(Chimera::GPIO::Port::PORTB, 3 ) );

    // Pin 14 is the first pin on PORTC
    const auto tmp3 = &( prjPortAttributes[ c ].pins[ 0 ] );
    EXPECT_EQ( tmp3, getPinAttributes( Chimera::GPIO::Port::PORTC, 14 ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getPortAttributes()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_GPIO_PortAttributes, InvalidInputs )
  {
    EXPECT_EQ( nullptr, getPortAttributes( Chimera::GPIO::Port::NUM_OPTIONS ) );
    EXPECT_EQ( nullptr, getPortAttributes( Chimera::GPIO::Port::PORTK ) );
  }

  TEST( Normal_GPIO_PortAttributes, ValidInputs )
  {
    const auto tmp1 = &prjPortAttributes[ static_cast<size_t>( Chimera::GPIO::Port::PORTA ) ];
    EXPECT_EQ( tmp1, getPortAttributes( Chimera::GPIO::Port::PORTA ) );

    const auto tmp2 = &prjPortAttributes[ static_cast<size_t>( Chimera::GPIO::Port::PORTB ) ];
    EXPECT_EQ( tmp2, getPortAttributes( Chimera::GPIO::Port::PORTB ) );

    const auto tmp3 = &prjPortAttributes[ static_cast<size_t>( Chimera::GPIO::Port::PORTC ) ];
    EXPECT_EQ( tmp3, getPortAttributes( Chimera::GPIO::Port::PORTC ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getPort()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_GPIO_GetPort, InvalidInputs )
  {
    EXPECT_EQ( Chimera::GPIO::Port::UNKNOWN_PORT, getPort( reinterpret_cast<std::uintptr_t>( nullptr ) ) );
    EXPECT_EQ( Chimera::GPIO::Port::UNKNOWN_PORT, getPort( std::numeric_limits<std::uintptr_t>::min() ) );
    EXPECT_EQ( Chimera::GPIO::Port::UNKNOWN_PORT, getPort( std::numeric_limits<std::uintptr_t>::max() ) );
  }

  TEST( Normal_GPIO_GetPort, ValidInputs )
  {
    EXPECT_EQ( Chimera::GPIO::Port::PORTA, getPort( reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ) ) );
    EXPECT_EQ( Chimera::GPIO::Port::PORTB, getPort( reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ) ) );
    EXPECT_EQ( Chimera::GPIO::Port::PORTC, getPort( reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ) ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: attachDriverInstances()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_GPIO_AttachDriver, InvalidInputs )
  {
    EXPECT_EQ( false, attachDriverInstances( nullptr, 1 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, 0 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, DRIVER_MAX_PORTS + 1 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, ARRAY_COUNT( fakeDriverList ) - 5 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, NUM_GPIO_PINS + 1 ) );
  }

  TEST( Normal_GPIO_AttachDriver, ValidInputs )
  {
    EXPECT_EQ( true, attachDriverInstances( fakeDriverList, ARRAY_COUNT( fakeDriverList ) ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: findAlternateFunction()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_GPIO_FindAlt, InvalidInputs )
  {
    // Bad port
    EXPECT_EQ( BAD_ALT_FUNC,
               findAlternateFunction( Chimera::GPIO::Port::UNKNOWN_PORT, 0, Chimera::GPIO::Alternate::TIM2_CH1 ) );

    // Bad pin
    EXPECT_EQ( BAD_ALT_FUNC, findAlternateFunction( Chimera::GPIO::Port::PORTA, 255, Chimera::GPIO::Alternate::TIM2_CH1 ) );

    // Bad alternate
    EXPECT_EQ( BAD_ALT_FUNC, findAlternateFunction( Chimera::GPIO::Port::PORTA, 0, Chimera::GPIO::Alternate::NUM_OPTIONS ) );

  }

  TEST( Normal_GPIO_FindAlt, ValidInputs )
  {
    EXPECT_EQ( AF1_TIM2, findAlternateFunction( Chimera::GPIO::Port::PORTA, 0, Chimera::GPIO::Alternate::TIM2_CH1 ) );
    EXPECT_EQ( AF10_QUADSPI,
               findAlternateFunction( Chimera::GPIO::Port::PORTA, 6, Chimera::GPIO::Alternate::QUADSPI_BK1_IO3 ) );
    EXPECT_EQ( AF9_TSC, findAlternateFunction( Chimera::GPIO::Port::PORTB, 4, Chimera::GPIO::Alternate::TSC_G2_IO1 ) );
    EXPECT_EQ( AF15_EVENTOUT, findAlternateFunction( Chimera::GPIO::Port::PORTC, 15, Chimera::GPIO::Alternate::EVENTOUT ) );
  }
}    // namespace Thor::Test::LLD

#endif /* THOR_LLD_TEST_GPIO_INTF */
