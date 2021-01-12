/********************************************************************************
 *  File Name:
 *    test_usart_intf.cpp
 *
 *  Description:
 *    Tests the interface layer for the low leve USART drivers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_prv_data.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_USART )

/* GTest Includes */
#include <gtest/gtest.h>

namespace Thor::LLD::USART
{
  static Driver fakeDriverList[ NUM_USART_PERIPHS ];


  /*-------------------------------------------------------------------------------
  FUT: isSupported()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_USART_IsSupported, InvalidInputs )
  {
    EXPECT_EQ( false, isSupported( Chimera::Serial::Channel::NOT_SUPPORTED ) );
    EXPECT_EQ( false, isSupported( Chimera::Serial::Channel::NUM_OPTIONS ) );
  }

  TEST( Normal_USART_IsSupported, ValidInputs )
  {
    EXPECT_EQ( true, isSupported( Chimera::Serial::Channel::SERIAL1 ) );
    EXPECT_EQ( true, isSupported( Chimera::Serial::Channel::SERIAL2 ) );
    EXPECT_EQ( false, isSupported( Chimera::Serial::Channel::SERIAL3 ) );
    EXPECT_EQ( false, isSupported( Chimera::Serial::Channel::SERIAL4 ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getResourceIndex()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_USART_ResourceIndex, InvalidInputs )
  {
    // Type 1: Chimera enum
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::Serial::Channel::NOT_SUPPORTED ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::Serial::Channel::NUM_OPTIONS ) );

    // Type 2: Peripheral Address
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( nullptr ) ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( std::numeric_limits<std::uintptr_t>::max() ) );
  }

  TEST( Normal_USART_ResourceIndex, ValidInputs )
  {
    // Type 1: Chimera enum
    EXPECT_EQ( USART1_RESOURCE_INDEX, getResourceIndex( Chimera::Serial::Channel::SERIAL1 ) );
    EXPECT_EQ( USART2_RESOURCE_INDEX, getResourceIndex( Chimera::Serial::Channel::SERIAL2 ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::Serial::Channel::SERIAL3 ) );
    EXPECT_EQ( INVALID_RESOURCE_INDEX, getResourceIndex( Chimera::Serial::Channel::SERIAL4 ) );

    // Type 2: Peripheral Address
    EXPECT_EQ( USART1_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( USART1_PERIPH ) ) );
    EXPECT_EQ( USART2_RESOURCE_INDEX, getResourceIndex( reinterpret_cast<std::uintptr_t>( USART2_PERIPH ) ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: getChannel()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_USART_GetChannel, InvalidInputs )
  {
    EXPECT_EQ( Chimera::Serial::Channel::NOT_SUPPORTED, getChannel( reinterpret_cast<std::uintptr_t>( nullptr ) ) );
    EXPECT_EQ( Chimera::Serial::Channel::NOT_SUPPORTED, getChannel( std::numeric_limits<std::uintptr_t>::max() ) );
  }

  TEST( Normal_USART_GetChannel, ValidInputs )
  {
    EXPECT_EQ( Chimera::Serial::Channel::SERIAL1, getChannel( reinterpret_cast<std::uintptr_t>( USART1_PERIPH ) ) );
    EXPECT_EQ( Chimera::Serial::Channel::SERIAL2, getChannel( reinterpret_cast<std::uintptr_t>( USART2_PERIPH ) ) );
  }


  /*-------------------------------------------------------------------------------
  FUT: attachDriverInstances()
  Range: Normal & Robust
  -------------------------------------------------------------------------------*/
  TEST( Robust_USART_AttachDriver, InvalidInputs )
  {
    EXPECT_EQ( false, attachDriverInstances( nullptr, 1 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, 0 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, NUM_USART_PERIPHS + 1 ) );
    EXPECT_EQ( false, attachDriverInstances( fakeDriverList, ARRAY_COUNT( fakeDriverList ) - 1 ) );
  }

  TEST( Normal_USART_AttachDriver, ValidInputs )
  {
    EXPECT_EQ( true, attachDriverInstances( fakeDriverList, ARRAY_COUNT( fakeDriverList ) ) );
  }

}    // namespace Thor::LLD::USART

#endif
